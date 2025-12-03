#include <unitree/robot/client/client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/String_.hpp>
#include <json.hpp>
#include <termio.h>
#include <string>
#include <future>
#include <thread>
#include <mutex>//互斥锁
#include <fstream>//判断pcd文件是否存在

#define SlamInfoTopic "rt/slam_info"
#define SlamKeyInfoTopic "rt/slam_key_info"
using namespace unitree::robot;
using namespace unitree::common;
unsigned char currentKey;

//定义poseDate结构体，用于存储位姿信息
class poseDate
{
public:  //public成员，可以在结构体外部访问
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float q_x = 0.0f;
    float q_y = 0.0f;
    float q_z = 0.0f;
    float q_w = 1.0f;
    int mode = 1;
    float speed = 0.8f;
    std::string toJsonStr() const
    {
        nlohmann::json j;
        j["data"]["targetPose"]["x"] = x;
        j["data"]["targetPose"]["y"] = y;
        j["data"]["targetPose"]["z"] = z;
        j["data"]["targetPose"]["q_x"] = q_x;
        j["data"]["targetPose"]["q_y"] = q_y;
        j["data"]["targetPose"]["q_z"] = q_z;
        j["data"]["targetPose"]["q_w"] = q_w;
        j["data"]["mode"] = mode;
        j["data"]["speed"] = speed;
        return j.dump(4);
    }
    void printInfo() const  
    //printInfo()是poseDate结构体的一个成员函数，用于打印当前位姿信息
    // const 修饰成员函数保证该函数不会修改调用对象（curPose）的任何成员变量”（如 x、y、z 等）
    {
        std::cout << "x:" << x << " y:" << y << " z:" << z << " q_x:"
                  << q_x << " q_y:" << q_y << " q_z:" << q_z << " q_w:" << q_w << std::endl;
    }
};

//slam命名空间
namespace unitree::robot::slam
{

    const std::string TEST_SERVICE_NAME = "slam_operate";
    const std::string TEST_API_VERSION = "1.0.0.1";

    const int32_t ROBOT_API_ID_STOP_NODE = 1901;
    const int32_t ROBOT_API_ID_START_MAPPING_PL = 1801;
    const int32_t ROBOT_API_ID_END_MAPPING_PL = 1802;
    const int32_t ROBOT_API_ID_START_RELOCATION_PL = 1804;
    const int32_t ROBOT_API_ID_POSE_NAV_PL = 1102;
    const int32_t ROBOT_API_ID_PAUSE_NAV = 1201;
    const int32_t ROBOT_API_ID_RESUME_NAV = 1202;

    class TestClient : public Client  //继承Client类
    {
    private:  //私有成员，只能在TestClient类的内部访问（例如keyExecute（））
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamInfo;
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamKeyInfo;

        void slamInfoHandler(const void *message);
        void slamKeyInfoHandler(const void *message);

        poseDate curPose; //定义了curPose变量，用于存储当前位姿信息，类型是poseDate结构体
        // std::vector<poseDate> poseList;// poseList 是 std::vector 模板的实例化对象（
        // 简而言之，std::vector<poseDate>规定了一个动态数组用来储存）poseDate，poselist就是用于存储多个导航点的位姿信息的数组名

        bool is_arrived = false;
        bool threadControl = false;
        std::future<void> futThread;
        std::promise<void> prom;
        std::thread controlThread;

        // 原有成员不变，新增以下4个变量
        int mapCount = 1;               // 建图计数器（从1开始，对应1.pcd、2.pcd）
        int currentMapId = 0;           // 当前选中的地图ID（0表示未选地图）

        // 分地图存储导航点：key=地图ID（1~n），value=该地图的导航点列表
        std::map<int, std::vector<poseDate>> mapPoseLists;  
        // 可选：存储地图ID与Pcd路径的对应关系（方便后续扩展，如加载地图时用）
        std::map<int, std::string> mapPcdPaths;  
        std::mutex poseListMutex; // 保护mapPoseLists读写的互斥锁

    public:
        TestClient();
        ~TestClient();

        void Init();
        unsigned char keyDetection();
        unsigned char keyExecute();
        void stopNodeFun();
        void startMappingPlFun();
        void endMappingPlFun(); //重点修改：生成递增Pcd文件名
        void relocationPlFun();
        void taskLoopFun(std::promise<void> &prom); // 重点修改：用当前地图的导航点
        void pauseNavFun();
        void resumeNavFun();
        void taskThreadRun();
        void taskThreadStop();
    };

    TestClient::TestClient() : Client(TEST_SERVICE_NAME, false)
    {
        subSlamInfo = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamInfoTopic)/*订阅指定话题*/);
        //绑定回调函数slamInfoHandler，当接收到消息时会调用该函数进行处理
        //std_msgs::msg::dds_::String_：表示消息的数据类型，这里是字符串类型
        subSlamInfo->InitChannel(std::bind(&unitree::robot::slam::TestClient::slamInfoHandler, this, std::placeholders::_1), 1);
        //std::bind(...)：将 slamInfoHandler 函数绑定为订阅者的 “回调函数”
        // 意思是：“只要 rt/slam_info 话题有新消息，就自动调用 this（当前 TestClient 对象）的 slamInfoHandler 函数，并把新消息作为参数传进去”
        subSlamKeyInfo = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamKeyInfoTopic));
        subSlamKeyInfo->InitChannel(std::bind(&unitree::robot::slam::TestClient::slamKeyInfoHandler, this, std::placeholders::_1), 1);
        std::cout << "***********************  Unitree SLAM Demo ***********************\n";
        std::cout << "---------------            q    w   !!!!            -----------------\n";
        std::cout << "---------------            a    s   d   f        -----------------\n";
        std::cout << "---------------            z    x                -----------------\n";
        std::cout << "------------------------------------------------------------------\n";
        std::cout << "------------------ q: Start mapping!!!         -------------------\n";
        std::cout << "------------------ w: End mapping              -------------------\n";
        std::cout << "------------------ a: Start relocation         -------------------\n";
        std::cout << "------------------ s: Add pose to task list    -------------------\n";
        std::cout << "------------------ d: Execute task list        -------------------\n";
        std::cout << "------------------ f: Clear task list          -------------------\n";
        std::cout << "------------------ z: Pause navigation         -------------------\n";
        std::cout << "------------------ x: Resume navigation        -------------------\n";
        std::cout << "---------------- Press any other key to stop SLAM ----------------\n";
        std::cout << "------------------------------------------------------------------\n";
        std::cout << "------------------------------------------------------------------\n";
        std::cout << "--------------- Press 'Ctrl + C' to exit the program -------------\n";
        std::cout << "------------------------------------------------------------------\n";
        std::cout << "------------------------------------------------------------------\n"
                  << std::endl;
    }

    TestClient::~TestClient()
    {
        stopNodeFun();
    }

    //初始化函数，注册API
    void TestClient::Init()
    {
        SetApiVersion(TEST_API_VERSION);

        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_POSE_NAV_PL);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_PAUSE_NAV);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_RESUME_NAV);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_STOP_NODE);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_START_MAPPING_PL);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_END_MAPPING_PL);
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_START_RELOCATION_PL);
    }
}

//启动导航任务线程
void unitree::robot::slam::TestClient::taskThreadRun()
{
    taskThreadStop();
    prom = std::promise<void>();//重置线程同步资源，清除旧线程的同步痕迹，为新线程准备干净的同步资源
    futThread = prom.get_future();//重置线程同步资源，清除旧线程的同步痕迹，为新线程准备干净的同步资源
    //创建新线程并执行导航任务
    controlThread = std::thread(&unitree::robot::slam::TestClient::taskLoopFun, this, std::ref(prom));
    controlThread.detach();
}

//导航任务线程函数（修改的主要部分）
/*void unitree::robot::slam::TestClient::taskLoopFun(std::promise<void> &prom)
{
    std::string data;
    threadControl = true;
    std::cout << "task list num:" << poseList.size() << std::endl;

    for (int i = 0; i < poseList.size(); i++)
    {
        is_arrived = false;

        //执行到第几个导航点，共多少个
        std::cout << "=== Start executing the " << (i + 1) << " th navigation point.（sum is " << poseList.size() << " ） ===" << std::endl;

        int32_t statusCode = Call(ROBOT_API_ID_POSE_NAV_PL, poseList[i].toJsonStr(), data);
        std::cout << "parameter:" << poseList[i].toJsonStr() << std::endl;
        std::cout << "statusCode:" << statusCode << std::endl;
        std::cout << "data:" << data << std::endl;

        while (!is_arrived)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (!threadControl)
                break;
        }

        if (i == poseList.size() - 1)
        {
            i = 0;
            std::reverse(poseList.begin(), poseList.end());
        }
        if (!threadControl)
            break;
    }

    prom.set_value();
}*/
void unitree::robot::slam::TestClient::taskLoopFun(std::promise<void> &prom)
{
    std::string data;
    threadControl = true;
    
    // 1. 再次检查当前地图和导航点（避免线程启动后地图切换）
    if (currentMapId == 0 || mapPoseLists[currentMapId].empty())
    {
        std::cout << "⚠️  导航失败：无有效地图或导航点" << std::endl;
        threadControl = false;
        prom.set_value();
        return;
    }
    
    // 2. 加锁拷贝“导航点快照”（关键修改：用快照代替引用）
    std::vector<poseDate> currentPoseList;  // 快照变量
    {
        std::lock_guard<std::mutex> lockNav(poseListMutex);  // 加锁读
        // 检查当前地图是否有导航点（原有逻辑移到锁内，避免判断后列表被清空）
        if (mapPoseLists[currentMapId].empty()) {
            std::cout << "⚠️  导航失败：地图" << currentMapId << "无导航点" << std::endl;
            threadControl = false;
            prom.set_value();
            return;
        }
        currentPoseList = mapPoseLists[currentMapId];  // 拷贝快照（脱离原列表）
    }

    // 3. 后续遍历“快照”（原有逻辑保留，仅改遍历对象）
    std::cout << "task list num:" << currentPoseList.size() << "（地图" << currentMapId << "）" << std::endl;
    std::vector<poseDate> navList = currentPoseList;  // 再拷贝一份用于反转（避免改快照原数据）
    for (int i = 0; i < navList.size() && threadControl; i++)
    {
        is_arrived = false;

        // 打印当前执行的导航点（标注地图ID）
        std::cout << "=== 地图" << currentMapId << "：开始执行第" << (i + 1) << "个导航点（共" <<  navList.size() << "个） ===" << std::endl;

        // 调用导航API（用当前地图的导航点）
        int32_t statusCode = Call(ROBOT_API_ID_POSE_NAV_PL, navList[i].toJsonStr(), data);
        std::cout << "parameter:" << navList[i].toJsonStr() << std::endl;
        std::cout << "statusCode:" << statusCode << std::endl;
        std::cout << "data:" << data << std::endl;

        // 等待导航到达（或停止信号）
        while (!is_arrived && threadControl)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        // 原有逻辑：如果是最后一个点，反转列表循环（保留，适配当前地图的列表）
        if (i == navList.size() - 1)
        {
            i = -1 ;
            std::reverse(navList.begin(), navList.end());
            std::cout << "=== 地图" << currentMapId << "：已到达最后一个点，反转导航顺序 ===" << std::endl;
        }
    }

    // 导航结束
    std::cout << "=== 地图" << currentMapId << "：导航线程结束 ===" << std::endl;
    threadControl = false;
    prom.set_value();
}

//停止导航任务线程
void unitree::robot::slam::TestClient::taskThreadStop()
{
    threadControl = false;//触发taskLoopFun函数内的停止条件
    is_arrived = true; //确保等待到达的循环能退出
    if (futThread.valid())
    {
        auto status = futThread.wait_for(std::chrono::seconds(2));
        if (status != std::future_status::ready)
        {
            std::cout << "⚠️  导航线程未及时退出，强制终止" << std::endl;
            // 若支持线程中断，可增加中断逻辑（需依赖Unitree SDK接口）
        }
             // 重置future，避免后续复用旧资源
        futThread = std::future<void>();
    }
    // 重置导航状态，避免残留
    is_arrived = false;
}

//处理slam信息的回调函数
void unitree::robot::slam::TestClient::slamInfoHandler(const void *message)
{
    // 1. 将接收到的消息（void*类型）转换为标准字符串消息类型
    std_msgs::msg::dds_::String_ currentMsg = *(std_msgs::msg::dds_::String_ *)message;
     // 2. 解析JSON格式的字符串消息（提取位姿数据）
    nlohmann::json jsonData = nlohmann::json ::parse(currentMsg.data());

    // errorCode检查消息是否有错误
    if (jsonData["errorCode"] != 0)
    {
        std::cout << "\033[33m" << jsonData["info"] << "\033[0m" << std::endl;
        return;
    }

    // pose 确认消息类型是“位姿信息”（type="pos_info"），然后更新curPose
    if (jsonData["type"] == "pos_info")
    {
        curPose.x = jsonData["data"]["currentPose"]["x"];
        curPose.y = jsonData["data"]["currentPose"]["y"];
        curPose.z = jsonData["data"]["currentPose"]["z"];
        curPose.q_x = jsonData["data"]["currentPose"]["q_x"];
        curPose.q_y = jsonData["data"]["currentPose"]["q_y"];
        curPose.q_z = jsonData["data"]["currentPose"]["q_z"];
        curPose.q_w = jsonData["data"]["currentPose"]["q_w"];
    }
}

//处理slam键盘信息的回调函数
void unitree::robot::slam::TestClient::slamKeyInfoHandler(const void *message)
{
    std_msgs::msg::dds_::String_ currentMsg = *(std_msgs::msg::dds_::String_ *)message;
    nlohmann::json jsonData = nlohmann::json ::parse(currentMsg.data());

    // errorCode
    if (jsonData["errorCode"] != 0)
    {
        std::cout << "\033[33m" << jsonData["info"] << "\033[0m" << std::endl;
        return;
    }

    // task_result
    if (jsonData["type"] == "task_result")
    {
        is_arrived = jsonData["data"]["is_arrived"];
        if (is_arrived)
        {
            std::cout << "I arrived " << jsonData["data"]["targetNodeName"] << std::endl;
        }
        else
        {
            std::cout << "I not arrived " << jsonData["data"]["targetNodeName"] << "  Please help me!!  (T_T)   (T_T)   (T_T) " << std::endl;
        }
    }
}

//停止slam服务
void unitree::robot::slam::TestClient::stopNodeFun()
{
    std::string parameter, data;
    parameter = R"({"data": {}})"; // Fixed data content
    int32_t statusCode = Call(ROBOT_API_ID_STOP_NODE, parameter, data);//关闭slam
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

//开始建图函数
void unitree::robot::slam::TestClient::startMappingPlFun()
{
    std::string parameter, data;
    parameter = R"({"data": {"slam_type": "indoor"}})"; // Fixed data content
    int32_t statusCode = Call(ROBOT_API_ID_START_MAPPING_PL, parameter, data);
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

//结束建图函数
/*void unitree::robot::slam::TestClient::endMappingPlFun()
{
    std::string parameter, data;
    parameter = R"({"data": {"address": "/home/unitree/test.pcd"}})"; // address:pcd file save address
    int32_t statusCode = Call(ROBOT_API_ID_END_MAPPING_PL, parameter, data);
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}*/
void unitree::robot::slam::TestClient::endMappingPlFun()
{
    std::string parameter, data;
    
    // 1. 生成递增的Pcd路径（/home/unitree/1.pcd、/home/unitree/2.pcd...）
    std::string pcdPath = "/home/unitree/" + std::to_string(mapCount) + ".pcd";
    // 2. 记录当前地图ID（mapCount）与Pcd路径的对应关系（后续选地图时可参考）
    mapPcdPaths[mapCount] = pcdPath;
    
    // 3. 构造建图参数（将递增的pcdPath传入JSON）
    // 注意：原始字符串R""()中，用{}包裹变量需拼接，不能直接写变量名
    parameter = R"({"data": {"address": ")" + pcdPath + R"("}})";
    
    // 4. 调用结束建图API
    int32_t statusCode = Call(ROBOT_API_ID_END_MAPPING_PL, parameter, data);
    
    // 5. 打印结果（新增：提示当前保存的Pcd文件名和对应地图ID）
    std::cout << "=== 建图结束 ===" << std::endl;
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
    
    // 6. 建图计数器+1（仅建图成功时）
    if (statusCode == 0) {  // 0为API调用成功码，
        mapPcdPaths[mapCount] = pcdPath;
        std::cout << "✅ 保存Pcd文件：" << pcdPath << std::endl;
        std::cout << "✅ 该地图ID：" << mapCount << "（按数字键" << mapCount << "选择此地图）" << std::endl;
        mapCount++;  // 仅成功时递增
    } else {
        std::cout << "❌ 建图失败，不记录地图ID" << std::endl;
    }
}

//开始定位函数
/*void unitree::robot::slam::TestClient::relocationPlFun()
{
    std::string parameter, data;
    parameter =
        R"({
        "data": {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "q_x": 0.0,
            "q_y": 0.0,
            "q_z": 0.0,
            "q_w": 1.0,
            "address": "/home/unitree/test.pcd"
        }
        })"; // x/y/z/q_x/q_y/q_z/q_w:Initialize pose information    address:pcd file reading address
    int32_t statusCode = Call(ROBOT_API_ID_START_RELOCATION_PL, parameter, data);
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}*/
// 开始定位函数（修改后：基于当前选中地图的Pcd定位）
void unitree::robot::slam::TestClient::relocationPlFun()
{
    std::string parameter, data;
    
    // 构造定位参数（传入当前地图的Pcd路径）
    if (mapPcdPaths.find(currentMapId) != mapPcdPaths.end())
    {
        std::string pcdPath = mapPcdPaths[currentMapId];
        // 校验文件是否存在
        if (!std::ifstream(pcdPath)) {  // 若文件不存在，if条件为true
            std::cout << "❌ 地图" << currentMapId << "的Pcd文件不存在：" << pcdPath << std::endl;
            return;  // 终止定位，避免无效调用
        }
        // 拼接JSON参数（包含当前地图的Pcd路径）
        parameter = R"({"data": {"x": 0.0,"y": 0.0,"z": 0.0,"q_x": 0.0,"q_y": 0.0,"q_z": 0.0,"q_w": 1.0,"address": ")" 
                    + mapPcdPaths[currentMapId] + R"("}})";
    }
    else
    {
        // 如果没有Pcd路径，用默认参数（兼容原有逻辑）
        parameter = R"({"data": {"x": 0.0,"y": 0.0,"z": 0.0,"q_x": 0.0,"q_y": 0.0,"q_z": 0.0,"q_w": 1.0,"address": "/home/unitree/test.pcd"}})";
    }

    // 调用定位API
    int32_t statusCode = Call(ROBOT_API_ID_START_RELOCATION_PL, parameter, data);
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "基于地图" << currentMapId << "定位，Pcd路径：" << (mapPcdPaths.count(currentMapId) ? mapPcdPaths[currentMapId] : "默认路径") << std::endl;
    std::cout << "data:" << data << std::endl;
}

//暂停导航函数
void unitree::robot::slam::TestClient::pauseNavFun()
{
    std::string parameter, data;
    parameter = R"({"data": {}})"; // Fixed data content
    int32_t statusCode = Call(ROBOT_API_ID_PAUSE_NAV, parameter, data);
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

//恢复导航函数
void unitree::robot::slam::TestClient::resumeNavFun()
{
    std::string parameter, data;
    parameter = R"({"data": {}})"; // Fixed data content
    int32_t statusCode = Call(ROBOT_API_ID_RESUME_NAV, parameter, data);
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

//键盘检测函数
unsigned char unitree::robot::slam::TestClient::keyDetection()
{
    termios tms_old, tms_new;
    tcgetattr(0, &tms_old);
    tms_new = tms_old;
    tms_new.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &tms_new);
    unsigned char ch = getchar();
    tcsetattr(0, TCSANOW, &tms_old);
    std::cout << "\033[1;32m"
              << "Key " << ch << " pressed."
              << "\033[0m" << std::endl;
    return ch;
}

//键盘控制执行函数
unsigned char unitree::robot::slam::TestClient::keyExecute()
{
    unsigned char currentKey;
    while (true)
    {
        currentKey = keyDetection();
        switch (currentKey)
        {
        case 'q':
            std::cout << "=== 开始建图（下次结束将保存为" << mapCount << ".pcd）===" << std::endl;
            startMappingPlFun();
            break;
        case 'w':
            endMappingPlFun();
            break;
        case 'a':
            // 定位时需关联当前选中的地图（如果有），可扩展参数传入currentMapId
            if (currentMapId == 0)
            {
                std::cout << "⚠️  请先按数字键1~9选择地图，再执行定位！" << std::endl;
                break;
            }
            std::cout << "=== 基于地图" << currentMapId << "执行定位 ===" << std::endl;
            relocationPlFun();
            break;

        // 新增：数字键1~9选择地图
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
             // 停止当前导航（避免切换地图时导航冲突）
            taskThreadStop();
            // 数字字符转成地图ID（如'1'→1，'2'→2）
            currentMapId = currentKey - '0';
            // 如果该地图ID未创建过导航点列表，初始化一个空列表
            if (mapPoseLists.find(currentMapId) == mapPoseLists.end())
            {
                mapPoseLists[currentMapId] = std::vector<poseDate>();
            }
            // 打印选中结果（提示当前地图的Pcd路径和导航点数量）
            std::cout << "=== 已选中地图" << currentMapId << " ===" << std::endl;
            if (mapPcdPaths.find(currentMapId) != mapPcdPaths.end())
            {
                std::cout << "对应Pcd文件：" << mapPcdPaths[currentMapId] << std::endl;
            }
            else
            {
                std::cout << "提示：该地图尚未保存Pcd文件（需先执行建图+结束建图）" << std::endl;
            }
            std::cout << "当前导航点数量：" << mapPoseLists[currentMapId].size() << std::endl;
            break;

        /*case 's':
            poseList.push_back(curPose);
            //push_back()函数用于在向量的末尾添加一个新的元素，这里是将当前位姿curPose添加到导航任务列表poseList中
            //curPose是前面定义的poseDate结构体类型的变量，存储了当前机器人的位姿信息 
            curPose.printInfo();//打印当前位姿（保存的）
            break;*/

         case 's':
         {
            // 检查是否已选地图
            if (currentMapId == 0)
            {
                std::cout << "⚠️  请先按数字键1~9选择地图，再按s存导航点！" << std::endl;
                break;
            }
            // 加锁：保护mapPoseLists的写操作（push_back）
            std::lock_guard<std::mutex> lockS(poseListMutex);  
            // 存当前位姿到当前地图的导航点列表
            mapPoseLists[currentMapId].push_back(curPose);
            // 打印结果（提示当前地图和导航点数量）
            std::cout << "=== 已保存导航点到地图" << currentMapId << " ===" << std::endl;
            curPose.printInfo();
            std::cout << "当前地图导航点总数：" << mapPoseLists[currentMapId].size() << std::endl;
            break;
         }
            
        /*case 'd':
            taskThreadRun();
            break;*/

        // 修改：d键→基于当前选中地图的导航点执行导航
        case 'd':
            // 检查是否已选地图
            if (currentMapId == 0)
            {
                std::cout << "⚠️  请先按数字键1~9选择地图，再按d执行导航！" << std::endl;
                break;
            }
            // 检查当前地图是否有导航点
            if (mapPoseLists[currentMapId].empty())
            {
                std::cout << "⚠️  地图" << currentMapId << "暂无导航点，请先按s存点！" << std::endl;
                break;
            }
            // 启动导航线程（后续修改taskLoopFun用当前地图的列表）
            std::cout << "=== 基于地图" << currentMapId << "执行导航（共" << mapPoseLists[currentMapId].size() << "个点）===" << std::endl;
            taskThreadRun();
            break;
        
        /*case 'f':
            poseList.clear();
            std::cout << "Clear task list" << std::endl;
            break;*/
        
            // 修改：f键→清空当前选中地图的导航点
        case 'f':
        {
            if (currentMapId == 0)
            {
                std::cout << "⚠️  请先按数字键1~9选择地图，再按f清空导航点！" << std::endl;
                break;
            }
            // 加锁：保护mapPoseLists的写操作（clear）
            std::lock_guard<std::mutex> lockF(poseListMutex); 
            mapPoseLists[currentMapId].clear();
            std::cout << "=== 已清空地图" << currentMapId << "的所有导航点 ===" << std::endl;
            break;
        }
        case 'z':
            pauseNavFun();
            break;
        case 'x':
            resumeNavFun();
            break;
        default:
            taskThreadStop();
            stopNodeFun();
            break;
        }
    }
}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]); // argv[1]：The name of the network card with network segment 123
    unitree::robot::slam::TestClient tc;

    tc.Init();
    tc.SetTimeout(10.0f);

    tc.keyExecute();
    return 0;
}
