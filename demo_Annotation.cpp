// 引入Unitree机器人客户端基础类头文件（提供客户端核心功能）
#include <unitree/robot/client/client.hpp>
// 引入Unitree机器人通信订阅者头文件（用于订阅话题消息）
#include <unitree/robot/channel/channel_subscriber.hpp>
// 引入Unitree ROS2 IDL字符串消息类型头文件（处理ROS2格式的字符串消息）
#include <unitree/idl/ros2/String_.hpp>
// 引入nlohmann JSON解析库头文件（用于JSON数据的序列化/反序列化）
#include <json.hpp>
// 引入termios头文件（用于修改终端模式，实现无回显键盘检测）
#include <termio.h>
// 引入字符串处理头文件（用于字符串操作）
#include <string>
// 引入future头文件（用于线程同步，获取线程执行结果）
#include <future>
// 引入thread头文件（用于多线程编程）
#include <thread>
// 引入mutex头文件（用于互斥锁，保证多线程数据安全）
#include <mutex>//互斥锁
// 引入fstream头文件（用于文件操作，此处判断PCD文件是否存在）
#include <fstream>//判断pcd文件是否存在

// 定义SLAM基础信息话题名（用于订阅SLAM的位姿、状态等基础信息）
#define SlamInfoTopic "rt/slam_info"
// 定义SLAM关键信息话题名（用于订阅SLAM的任务结果、关键事件等信息）
#define SlamKeyInfoTopic "rt/slam_key_info"
// 使用Unitree机器人命名空间（简化代码，避免重复写命名空间前缀）
using namespace unitree::robot;
// 使用Unitree通用命名空间（简化通用工具类调用）
using namespace unitree::common;
// 定义全局变量currentKey（存储当前按下的键盘按键，unsigned char避免符号问题）
unsigned char currentKey;

//定义poseDate结构体，用于存储机器人的位姿（位置+姿态）和导航参数信息
class poseDate
{
public:  //public成员，可以在结构体外部访问（允许外部读写位姿和导航参数）
    // 机器人位置坐标：x轴（单位：米），初始值0.0
    float x = 0.0f;
    // 机器人位置坐标：y轴（单位：米），初始值0.0
    float y = 0.0f;
    // 机器人位置坐标：z轴（单位：米，平面导航时通常为0），初始值0.0
    float z = 0.0f;
    // 机器人姿态四元数：x分量（用于表示3D姿态），初始值0.0
    float q_x = 0.0f;
    // 机器人姿态四元数：y分量，初始值0.0
    float q_y = 0.0f;
    // 机器人姿态四元数：z分量，初始值0.0
    float q_z = 0.0f;
    // 机器人姿态四元数：w分量（实部），初始值1.0（表示初始无旋转）
    float q_w = 1.0f;
    // 导航模式（1通常为默认模式，具体含义由机器人SDK定义），初始值1
    int mode = 1;
    // 导航速度（单位：米/秒），初始值0.8
    float speed = 0.8f;

    // 成员函数：将当前位姿和导航参数转换为JSON字符串（用于API调用传参）
    // const修饰：保证该函数不修改poseDate对象的任何成员变量
    std::string toJsonStr() const
    {
        // 创建JSON对象j
        nlohmann::json j;
        // 向JSON中添加位置信息（x/y/z）
        j["data"]["targetPose"]["x"] = x;
        j["data"]["targetPose"]["y"] = y;
        j["data"]["targetPose"]["z"] = z;
        // 向JSON中添加姿态信息（四元数x/y/z/w）
        j["data"]["targetPose"]["q_x"] = q_x;
        j["data"]["targetPose"]["q_y"] = q_y;
        j["data"]["targetPose"]["q_z"] = q_z;
        j["data"]["targetPose"]["q_w"] = q_w;
        // 向JSON中添加导航模式
        j["data"]["mode"] = mode;
        // 向JSON中添加导航速度
        j["data"]["speed"] = speed;
        // 将JSON对象转为字符串，缩进4个空格（便于阅读）并返回
        return j.dump(4);
    }

    // 成员函数：打印当前位姿信息（x/y/z + 四元数）
    // const修饰：保证该函数不修改poseDate对象的任何成员变量
    void printInfo() const  
    {
        // 输出位姿数据到控制台，格式为"x:值 y:值 ... q_w:值"
        std::cout << "x:" << x << " y:" << y << " z:" << z << " q_x:"
                  << q_x << " q_y:" << q_y << " q_z:" << q_z << " q_w:" << q_w << std::endl;
    }
};

// 定义SLAM功能专属命名空间（隔离SLAM相关类和常量，避免命名冲突）
namespace unitree::robot::slam
{
    // 定义SLAM服务名称（用于与机器人SLAM模块建立服务通信）
    const std::string TEST_SERVICE_NAME = "slam_operate";
    // 定义SLAM API版本号（与机器人SLAM模块版本匹配，保证兼容性）
    const std::string TEST_API_VERSION = "1.0.0.1";

    // 定义SLAM功能API ID（每个ID对应一个具体功能，由Unitree SDK约定）
    const int32_t ROBOT_API_ID_STOP_NODE = 1901;       // 停止SLAM节点
    const int32_t ROBOT_API_ID_START_MAPPING_PL = 1801;// 开始平面建图
    const int32_t ROBOT_API_ID_END_MAPPING_PL = 1802;  // 结束平面建图
    const int32_t ROBOT_API_ID_START_RELOCATION_PL = 1804;// 开始平面重定位
    const int32_t ROBOT_API_ID_POSE_NAV_PL = 1102;     // 平面位姿导航
    const int32_t ROBOT_API_ID_PAUSE_NAV = 1201;       // 暂停导航
    const int32_t ROBOT_API_ID_RESUME_NAV = 1202;      // 恢复导航

    // 定义TestClient类，继承自Client（Unitree机器人客户端基类）
    // 作用：封装SLAM相关的所有功能（订阅、API调用、键盘控制等）
    class TestClient : public Client  
    {
    private:  // 私有成员：仅类内部可访问（保证数据安全性）
        // 定义SLAM基础信息订阅者指针（订阅SlamInfoTopic话题，消息类型为ROS2字符串）
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamInfo;
        // 定义SLAM关键信息订阅者指针（订阅SlamKeyInfoTopic话题，消息类型为ROS2字符串）
        ChannelSubscriberPtr<std_msgs::msg::dds_::String_> subSlamKeyInfo;

        // 私有成员函数：SLAM基础信息回调函数（收到SlamInfoTopic消息时触发）
        void slamInfoHandler(const void *message);
        // 私有成员函数：SLAM关键信息回调函数（收到SlamKeyInfoTopic消息时触发）
        void slamKeyInfoHandler(const void *message);

        // 定义curPose变量：存储机器人当前位姿信息（类型为poseDate结构体）
        poseDate curPose; 
        // 【已注释】原导航点列表（单地图版本），现改为分地图存储
        // std::vector<poseDate> poseList;

        // 导航状态标志：是否到达目标点（由slamKeyInfoHandler更新）
        bool is_arrived = false;
        // 线程控制标志：控制导航线程的启动/停止（true=运行，false=停止）
        bool threadControl = false;
        // future对象：用于获取导航线程的执行结果（同步线程）
        std::future<void> futThread;
        // promise对象：用于设置导航线程的执行结果（与futThread配对）
        std::promise<void> prom;
        // 导航线程对象：执行导航任务的独立线程
        std::thread controlThread;

        // 【新增成员变量】：支持多地图管理
        int mapCount = 1;               // 建图计数器（从1开始，对应1.pcd、2.pcd等递增文件名）
        int currentMapId = 0;           // 当前选中的地图ID（0表示未选地图，1~9对应数字键选择）
        std::map<int, std::vector<poseDate>> mapPoseLists;  // 分地图存储导航点：key=地图ID，value=该地图的导航点列表
        std::map<int, std::string> mapPcdPaths;  // 地图ID与PCD文件路径的映射（便于定位时加载对应地图）
        std::mutex poseListMutex; // 互斥锁：保护mapPoseLists的读写操作（避免多线程数据竞争）

    public:  // 公有成员：类外部可访问（提供功能接口）
        TestClient();  // 构造函数：初始化订阅者、打印操作提示
        ~TestClient(); // 析构函数：释放资源（停止SLAM节点）

        void Init();  // 初始化函数：注册SLAM API接口
        unsigned char keyDetection();  // 键盘检测函数：获取按下的按键（无回显）
        unsigned char keyExecute();    // 键盘处理函数：根据按键执行对应SLAM功能
        void stopNodeFun();            // 停止SLAM节点函数
        void startMappingPlFun();      // 开始平面建图函数
        void endMappingPlFun();        // 结束平面建图函数（重点修改：递增PCD文件名）
        void relocationPlFun();        // 开始平面重定位函数（重点修改：关联当前地图）
        void taskLoopFun(std::promise<void> &prom); // 导航线程核心函数（重点修改：用当前地图导航点）
        void pauseNavFun();            // 暂停导航函数
        void resumeNavFun();           // 恢复导航函数
        void taskThreadRun();          // 启动导航线程函数
        void taskThreadStop();         // 停止导航线程函数
    };

    // TestClient构造函数：初始化父类Client，创建订阅者，绑定回调，打印操作提示
    TestClient::TestClient() : Client(TEST_SERVICE_NAME, false)
    {
        // 1. 创建SLAM基础信息订阅者：订阅SlamInfoTopic话题
        subSlamInfo = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(
            new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamInfoTopic)
        );
        // 初始化订阅者通道：绑定回调函数slamInfoHandler，队列大小1（只缓存最新消息）
        subSlamInfo->InitChannel(
            std::bind(&unitree::robot::slam::TestClient::slamInfoHandler, this, std::placeholders::_1), 
            1
        );

        // 2. 创建SLAM关键信息订阅者：订阅SlamKeyInfoTopic话题
        subSlamKeyInfo = ChannelSubscriberPtr<std_msgs::msg::dds_::String_>(
            new ChannelSubscriber<std_msgs::msg::dds_::String_>(SlamKeyInfoTopic)
        );
        // 初始化订阅者通道：绑定回调函数slamKeyInfoHandler，队列大小1
        subSlamKeyInfo->InitChannel(
            std::bind(&unitree::robot::slam::TestClient::slamKeyInfoHandler, this, std::placeholders::_1), 
            1
        );

        // 3. 打印SLAM Demo操作提示（告诉用户各按键功能）
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

    // TestClient析构函数：对象销毁时停止SLAM节点，释放资源
    TestClient::~TestClient()
    {
        stopNodeFun(); // 调用停止SLAM节点函数
    }

    // TestClient初始化函数：设置API版本，注册SLAM相关API接口
    void TestClient::Init()
    {
        // 设置SLAM API版本（与机器人SLAM模块版本匹配）
        SetApiVersion(TEST_API_VERSION);

        // 注册SLAM功能API（无输入参数类型，由宏UT_ROBOT_CLIENT_REG_API_NO_PROI处理）
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_POSE_NAV_PL);    // 注册位姿导航API
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_PAUSE_NAV);      // 注册暂停导航API
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_RESUME_NAV);     // 注册恢复导航API
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_STOP_NODE);      // 注册停止SLAM节点API
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_START_MAPPING_PL);// 注册开始建图API
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_END_MAPPING_PL);  // 注册结束建图API
        UT_ROBOT_CLIENT_REG_API_NO_PROI(ROBOT_API_ID_START_RELOCATION_PL);// 注册开始重定位API
    }
}

// 实现TestClient的启动导航线程函数：停止旧线程，创建新导航线程
void unitree::robot::slam::TestClient::taskThreadRun()
{
    taskThreadStop(); // 先停止已有的导航线程（避免多线程冲突）
    prom = std::promise<void>(); // 重置promise对象（清除旧线程的同步状态）
    futThread = prom.get_future(); // 重置futThread，与新promise配对
    // 创建新导航线程：执行taskLoopFun函数，传入promise引用（用于线程同步）
    controlThread = std::thread(&unitree::robot::slam::TestClient::taskLoopFun, this, std::ref(prom));
    controlThread.detach(); // 线程分离：不需要等待线程结束（由threadControl控制停止）
}

// 【已注释】原导航线程函数（单地图版本）
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

// 实现TestClient的导航线程核心函数（多地图版本）：执行当前地图的导航任务
void unitree::robot::slam::TestClient::taskLoopFun(std::promise<void> &prom)
{
    std::string data; // 存储API调用的返回数据
    threadControl = true; // 设置线程控制标志为true（线程开始运行）
    
    // 1. 检查当前地图状态：无选中地图或地图无导航点，导航失败
    if (currentMapId == 0 || mapPoseLists[currentMapId].empty())
    {
        std::cout << "⚠️  导航失败：无有效地图或导航点" << std::endl;
        threadControl = false; // 停止线程
        prom.set_value(); // 设置promise结果（通知主线程线程结束）
        return;
    }
    
    // 2. 加锁拷贝当前地图的导航点快照（避免导航中列表被修改，保证线程安全）
    std::vector<poseDate> currentPoseList;  // 导航点快照（拷贝后独立于原列表）
    {
        // 创建互斥锁守卫：自动加锁，作用域结束自动解锁（避免死锁）
        std::lock_guard<std::mutex> lockNav(poseListMutex);  
        // 再次检查当前地图导航点（防止加锁前列表被清空）
        if (mapPoseLists[currentMapId].empty()) {
            std::cout << "⚠️  导航失败：地图" << currentMapId << "无导航点" << std::endl;
            threadControl = false;
            prom.set_value();
            return;
        }
        currentPoseList = mapPoseLists[currentMapId];  // 拷贝导航点到快照
    }

    // 3. 执行导航任务（遍历快照中的导航点）
    std::cout << "task list num:" << currentPoseList.size() << "（地图" << currentMapId << "）" << std::endl;
    std::vector<poseDate> navList = currentPoseList;  // 再拷贝一份用于反转（不修改原快照）
    // 循环遍历导航点，threadControl为false时停止
    for (int i = 0; i < navList.size() && threadControl; i++)
    {
        is_arrived = false; // 重置到达标志（准备前往下一个点）

        // 打印当前导航点信息（地图ID、序号、总数）
        std::cout << "=== 地图" << currentMapId << "：开始执行第" << (i + 1) << "个导航点（共" <<  navList.size() << "个） ===" << std::endl;

        // 调用位姿导航API：传入当前导航点的JSON参数，获取返回状态和数据
        int32_t statusCode = Call(ROBOT_API_ID_POSE_NAV_PL, navList[i].toJsonStr(), data);
        std::cout << "parameter:" << navList[i].toJsonStr() << std::endl; // 打印传入参数
        std::cout << "statusCode:" << statusCode << std::endl; // 打印API调用状态码（0=成功）
        std::cout << "data:" << data << std::endl; // 打印API返回数据

        // 等待导航到达目标点（或线程停止信号）
        while (!is_arrived && threadControl)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(5)); // 休眠5ms，降低CPU占用
        }

        // 导航到最后一个点后，反转列表，循环导航（如：1→2→3 → 3→2→1→...）
        if (i == navList.size() - 1)
        {
            i = -1 ; // 下一轮从0开始（因循环结束会i++）
            //将指定范围内的元素反转
            std::reverse(navList.begin(), navList.end()); // 反转导航点顺序
            std::cout << "=== 地图" << currentMapId << "：已到达最后一个点，反转导航顺序 ===" << std::endl;
        }
    }

    // 导航线程结束：打印信息，重置标志，通知主线程
    std::cout << "=== 地图" << currentMapId << "：导航线程结束 ===" << std::endl;
    threadControl = false;
    prom.set_value();
}

// 实现TestClient的停止导航线程函数：停止当前导航线程
void unitree::robot::slam::TestClient::taskThreadStop()
{
    threadControl = false; // 设置线程控制标志为false（触发导航线程退出）
    // 检查futThread是否有效（避免重复等待）
    if (futThread.valid())
    {
        // 非阻塞检查线程是否已结束（等待0ms）
        auto status = futThread.wait_for(std::chrono::milliseconds(0));
        // 线程未结束则阻塞等待，直到线程完成
        if (status != std::future_status::ready)
            futThread.wait();
    }
}

// 实现TestClient的SLAM基础信息回调函数：处理位姿信息
void unitree::robot::slam::TestClient::slamInfoHandler(const void *message)
{
    // 1. 将void*类型的消息转换为ROS2字符串消息类型（SlamInfoTopic的消息格式）
    std_msgs::msg::dds_::String_ currentMsg = *(std_msgs::msg::dds_::String_ *)message;
    // 2. 解析JSON格式的消息数据（提取位姿等信息）
    nlohmann::json jsonData = nlohmann::json ::parse(currentMsg.data());

    // 3. 检查消息错误码：非0表示消息异常，打印警告信息并返回
    if (jsonData["errorCode"] != 0)
    {
        std::cout << "\033[33m" << jsonData["info"] << "\033[0m" << std::endl; // 33m=黄色字体，0m=恢复默认
        return;
    }

    // 4. 处理位姿信息：消息类型为"pos_info"时，更新curPose（当前位姿）
    if (jsonData["type"] == "pos_info")
    {
        curPose.x = jsonData["data"]["currentPose"]["x"];     // 更新x坐标
        curPose.y = jsonData["data"]["currentPose"]["y"];     // 更新y坐标
        curPose.z = jsonData["data"]["currentPose"]["z"];     // 更新z坐标
        curPose.q_x = jsonData["data"]["currentPose"]["q_x"]; // 更新四元数x
        curPose.q_y = jsonData["data"]["currentPose"]["q_y"]; // 更新四元数y
        curPose.q_z = jsonData["data"]["currentPose"]["q_z"]; // 更新四元数z
        curPose.q_w = jsonData["data"]["currentPose"]["q_w"]; // 更新四元数w
    }
}

// 实现TestClient的SLAM关键信息回调函数：处理导航结果
void unitree::robot::slam::TestClient::slamKeyInfoHandler(const void *message)
{
    // 1. 将void*消息转换为ROS2字符串消息类型（SlamKeyInfoTopic的消息格式）
    std_msgs::msg::dds_::String_ currentMsg = *(std_msgs::msg::dds_::String_ *)message;
    // 2. 解析JSON格式的消息数据
    nlohmann::json jsonData = nlohmann::json ::parse(currentMsg.data());

    // 3. 检查消息错误码：非0表示消息异常，打印警告信息并返回
    if (jsonData["errorCode"] != 0)
    {
        std::cout << "\033[33m" << jsonData["info"] << "\033[0m" << std::endl;
        return;
    }

    // 4. 处理导航任务结果：消息类型为"task_result"时，更新is_arrived（到达标志）
    if (jsonData["type"] == "task_result")
    {
        is_arrived = jsonData["data"]["is_arrived"]; // 从消息中获取到达状态
        // 根据到达状态打印不同信息
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

// 实现TestClient的停止SLAM节点函数：调用API停止SLAM模块
void unitree::robot::slam::TestClient::stopNodeFun()
{
    std::string parameter, data; // parameter=API输入参数，data=API返回数据
    // 构造停止SLAM节点的参数（空JSON对象，因该API无需额外参数）
    parameter = R"({"data": {}})"; 
    // 调用停止SLAM节点API：传入API ID、参数，获取状态码和返回数据
    int32_t statusCode = Call(ROBOT_API_ID_STOP_NODE, parameter, data);
    // 打印API调用结果
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

// 实现TestClient的开始平面建图函数：调用API启动建图
void unitree::robot::slam::TestClient::startMappingPlFun()
{
    std::string parameter, data;
    // 构造开始建图的参数：指定建图类型为"indoor"（室内建图）
    parameter = R"({"data": {"slam_type": "indoor"}})"; 
    // 调用开始平面建图API
    int32_t statusCode = Call(ROBOT_API_ID_START_MAPPING_PL, parameter, data);
    // 打印API调用结果
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

// 【已注释】原结束平面建图函数（单地图版本，固定PCD文件名）
/*void unitree::robot::slam::TestClient::endMappingPlFun()
{
    std::string parameter, data;
    parameter = R"({"data": {"address": "/home/unitree/test.pcd"}})"; // address:pcd file save address
    int32_t statusCode = Call(ROBOT_API_ID_END_MAPPING_PL, parameter, data);
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}*/

// 实现TestClient的结束平面建图函数（多地图版本，递增PCD文件名）
void unitree::robot::slam::TestClient::endMappingPlFun()
{
    std::string parameter, data;
    
    // 1. 生成递增的PCD文件路径（如mapCount=1→/home/unitree/1.pcd）
    std::string pcdPath = "/home/unitree/" + std::to_string(mapCount) + ".pcd";
    // 2. 记录当前地图ID（mapCount）与PCD路径的映射（用于后续定位）
    mapPcdPaths[mapCount] = pcdPath;
    
    // 3. 构造结束建图的参数：拼接PCD路径到JSON中（R""()为C++原始字符串，避免转义）
    parameter = R"({"data": {"address": ")" + pcdPath + R"("}})";
    
    // 4. 调用结束平面建图API
    int32_t statusCode = Call(ROBOT_API_ID_END_MAPPING_PL, parameter, data);
    
    // 5. 打印建图结果（包含PCD文件名和地图ID）
    std::cout << "=== 建图结束 ===" << std::endl;
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
    
    // 6. 建图成功则递增计数器（statusCode=0表示API调用成功）
    if (statusCode == 0) {  
        mapPcdPaths[mapCount] = pcdPath; // 再次赋值确保映射正确（冗余保护）
        std::cout << "✅ 保存Pcd文件：" << pcdPath << std::endl;
        std::cout << "✅ 该地图ID：" << mapCount << "（按数字键" << mapCount << "选择此地图）" << std::endl;
        mapCount++;  // 计数器递增（下次建图用下一个序号）
    } else {
        std::cout << "❌ 建图失败，不记录地图ID" << std::endl;
    }
}

// 【已注释】原开始平面重定位函数（单地图版本，固定PCD路径）
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

// 实现TestClient的开始平面重定位函数（多地图版本，用当前选中地图的PCD）
void unitree::robot::slam::TestClient::relocationPlFun()
{
    std::string parameter, data;
    
    // 1. 构造重定位参数：优先使用当前地图的PCD路径
    if (mapPcdPaths.find(currentMapId) != mapPcdPaths.end())
    {
        std::string pcdPath = mapPcdPaths[currentMapId];
        // 检查PCD文件是否存在（避免加载不存在的文件）
        if (!std::ifstream(pcdPath)) {  // ifstream打开失败表示文件不存在
            std::cout << "❌ 地图" << currentMapId << "的Pcd文件不存在：" << pcdPath << std::endl;
            return;  // 终止重定位，避免无效API调用
        }
        // 拼接重定位参数：包含初始位姿（0,0,0, 0,0,0,1）和当前地图PCD路径
        parameter = R"({"data": {"x": 0.0,"y": 0.0,"z": 0.0,"q_x": 0.0,"q_y": 0.0,"q_z": 0.0,"q_w": 1.0,"address": ")" 
                    + mapPcdPaths[currentMapId] + R"("}})";
    }
    else
    {
        // 无当前地图PCD路径时，使用默认路径（兼容旧逻辑）
        std::cout << "ℹ️  当前地图ID不存在，使用默认路径兼容旧逻辑" << std::endl;
        parameter = R"({"data": {"x": 0.0,"y": 0.0,"z": 0.0,"q_x": 0.0,"q_y": 0.0,"q_z": 0.0,"q_w": 1.0,"address": "/home/unitree/test.pcd"}})";
    }

    // 2. 调用开始重定位API
    int32_t statusCode = Call(ROBOT_API_ID_START_RELOCATION_PL, parameter, data);
    // 3. 打印重定位结果（包含地图ID和PCD路径）
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "基于地图" << currentMapId << "定位，Pcd路径：" << (mapPcdPaths.count(currentMapId) ? mapPcdPaths[currentMapId] : "默认路径") << std::endl;
    std::cout << "data:" << data << std::endl;
}

// 实现TestClient的暂停导航函数：调用API暂停当前导航
void unitree::robot::slam::TestClient::pauseNavFun()
{
    std::string parameter, data;
    // 构造暂停导航的参数（空JSON，无需额外参数）
    parameter = R"({"data": {}})"; 
    // 调用暂停导航API
    int32_t statusCode = Call(ROBOT_API_ID_PAUSE_NAV, parameter, data);
    // 打印API调用结果
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

// 实现TestClient的恢复导航函数：调用API恢复暂停的导航
void unitree::robot::slam::TestClient::resumeNavFun()
{
    std::string parameter, data;
    // 构造恢复导航的参数（空JSON，无需额外参数）
    parameter = R"({"data": {}})"; 
    // 调用恢复导航API
    int32_t statusCode = Call(ROBOT_API_ID_RESUME_NAV, parameter, data);
    // 打印API调用结果
    std::cout << "statusCode:" << statusCode << std::endl;
    std::cout << "data:" << data << std::endl;
}

// 实现TestClient的键盘检测函数：无回显获取按下的按键
unsigned char unitree::robot::slam::TestClient::keyDetection()
{
    termios tms_old, tms_new; // tms_old=原终端属性，tms_new=新终端属性
    tcgetattr(0, &tms_old);   // 获取标准输入（0表示stdin）的当前终端属性
    tms_new = tms_old;        // 复制原属性到新属性（基于原属性修改）
    
    // 修改新终端属性：关闭规范模式（ICANON）和回显（ECHO）
    // 规范模式关闭：按键无需按回车立即生效；回显关闭：按键不显示在控制台
    tms_new.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &tms_new); // 立即应用新终端属性（TCSANOW=立即生效）
    
    unsigned char ch = getchar(); // 读取按下的按键（无回显，无需回车）
    
    tcsetattr(0, TCSANOW, &tms_old); // 恢复原终端属性（避免影响后续终端操作）
    
    // 打印按键信息（绿色字体）
    std::cout << "\033[1;32m"
              << "Key " << ch << " pressed."
              << "\033[0m" << std::endl;
    return ch; // 返回按下的按键
}

// 实现TestClient的键盘处理函数：循环检测按键，执行对应功能
unsigned char unitree::robot::slam::TestClient::keyExecute()
{
    unsigned char currentKey; // 存储当前按下的按键
    while (true) // 无限循环：持续检测键盘输入
    {
        currentKey = keyDetection(); // 获取按下的按键
        // 根据按键执行对应功能（switch case分支）
        switch (currentKey)
        {
        case 'q': // 按下q键：开始建图
            std::cout << "=== 开始建图（下次结束将保存为" << mapCount << ".pcd）===" << std::endl;
            startMappingPlFun(); // 调用开始建图函数
            break;
        case 'w': // 按下w键：结束建图
            endMappingPlFun(); // 调用结束建图函数
            break;
        case 'a': // 按下a键：开始重定位
            // 检查是否已选地图：未选则提示
            if (currentMapId == 0)
            {
                std::cout << "⚠️  请先按数字键1~9选择地图，再执行定位！" << std::endl;
                break;
            }
            std::cout << "=== 基于地图" << currentMapId << "执行定位 ===" << std::endl;
            relocationPlFun(); // 调用开始重定位函数
            break;

        // 【新增】数字键1~9：选择对应ID的地图
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
             taskThreadStop(); // 停止当前导航（避免切换地图时导航冲突）
             // 数字字符转地图ID：如'1'的ASCII码是49，减'0'（48）得1
             currentMapId = currentKey - '0';
             // 若该地图ID未初始化导航点列表，则创建空列表
             if (mapPoseLists.find(currentMapId) == mapPoseLists.end())
             {
                 mapPoseLists[currentMapId] = std::vector<poseDate>();
             }
             // 打印地图选择结果
             std::cout << "=== 已选中地图" << currentMapId << " ===" << std::endl;
             // 打印该地图的PCD路径（若存在）
             if (mapPcdPaths.find(currentMapId) != mapPcdPaths.end())
             {
                 std::cout << "对应Pcd文件：" << mapPcdPaths[currentMapId] << std::endl;
             }
             else
             {
                 std::cout << "提示：该地图尚未保存Pcd文件（需先执行建图+结束建图）" << std::endl;
             }
             // 打印该地图的导航点数量
             std::cout << "当前导航点数量：" << mapPoseLists[currentMapId].size() << std::endl;
             break;

        // 【已注释】原添加导航点函数（单地图版本）
        /*case 's':
            poseList.push_back(curPose);
            //push_back()函数用于在向量的末尾添加一个新的元素，这里是将当前位姿curPose添加到导航任务列表poseList中
            //curPose是前面定义的poseDate结构体类型的变量，存储了当前机器人的位姿信息 
            curPose.printInfo();//打印当前位姿（保存的）
            break;*/

        case 's': // 按下s键：添加当前位姿到当前地图的导航点列表
            // 检查是否已选地图：未选则提示
            if (currentMapId == 0)
            {
                std::cout << "⚠️  请先按数字键1~9选择地图，再按s存导航点！" << std::endl;
                break;
            }
            // 加锁：保护mapPoseLists的写操作（避免多线程冲突）
            std::lock_guard<std::mutex> lockS(poseListMutex);  
            // 将当前位姿curPose添加到当前地图的导航点列表
            mapPoseLists[currentMapId].push_back(curPose);
            // 打印添加结果
            std::cout << "=== 已保存导航点到地图" << currentMapId << " ===" << std::endl;
            curPose.printInfo(); // 打印保存的位姿
            // 打印当前地图的导航点总数
            std::cout << "当前地图导航点总数：" << mapPoseLists[currentMapId].size() << std::endl;
            break;
        

        // 【已注释】原执行导航函数（单地图版本）
        /*case 'd':
            taskThreadRun();
            break;*/

        case 'd': // 按下d键：执行当前地图的导航任务
            // 检查是否已选地图：未选则提示
            if (currentMapId == 0)
            {
                std::cout << "⚠️  请先按数字键1~9选择地图，再按d执行导航！" << std::endl;
                break;
            }
            // 检查当前地图是否有导航点：无则提示
            if (mapPoseLists[currentMapId].empty())
            {
                std::cout << "⚠️  地图" << currentMapId << "暂无导航点，请先按s存点！" << std::endl;
                break;
            }
            // 打印导航启动信息
            std::cout << "=== 基于地图" << currentMapId << "执行导航（共" << mapPoseLists[currentMapId].size() << "个点）===" << std::endl;
            taskThreadRun(); // 启动导航线程
            break;
        
        // 【已注释】原清空导航点函数（单地图版本）
        /*case 'f':
            poseList.clear();
            std::cout << "Clear task list" << std::endl;
            break;*/
        
        case 'f': // 按下f键：清空当前地图的导航点列表
            // 检查是否已选地图：未选则提示
            if (currentMapId == 0)
            {
                std::cout << "⚠️  请先按数字键1~9选择地图，再按f清空导航点！" << std::endl;
                break;
            }
            // 加锁：保护mapPoseLists的写操作
            std::lock_guard<std::mutex> lockF(poseListMutex); 
            mapPoseLists[currentMapId].clear(); // 清空当前地图的导航点列表
            // 打印清空结果
            std::cout << "=== 已清空地图" << currentMapId << "的所有导航点 ===" << std::endl;
            break;
        case 'z': // 按下z键：暂停导航
            pauseNavFun(); // 调用暂停导航函数
            break;
        case 'x': // 按下x键：恢复导航
            resumeNavFun(); // 调用恢复导航函数
            break;
        default: // 按下其他键：停止导航和SLAM，退出循环
            taskThreadStop(); // 停止导航线程
            stopNodeFun();    // 停止SLAM节点
            break;
        }
    }
}

// 主函数：程序入口，初始化通信、创建SLAM客户端、启动键盘控制
int main(int argc, const char **argv)
{
    // 1. 检查命令行参数：需传入网卡名（如eth0），否则提示用法并退出
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1); // 异常退出（返回-1表示参数错误）
    }

    // 2. 初始化机器人通信通道：
    // ChannelFactory::Instance()：获取通信工厂单例（全局唯一）
    // Init(0, argv[1])：0=默认线程池大小，argv[1]=123网段的网卡名（用于SLAM通信）
    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]); 

    // 3. 创建SLAM客户端对象（触发TestClient构造函数）
    //从命名空间找到类，再实例化类对象
    unitree::robot::slam::TestClient tc;

    // 4. 初始化SLAM客户端：注册API接口
    tc.Init();
    // 设置API调用超时时间：10秒（超过10秒无响应则判定为调用失败）
    tc.SetTimeout(10.0f);

    // 5. 启动键盘控制循环（程序进入交互模式，直到按下非预设键退出）
    tc.keyExecute();

    // 6. 程序正常退出（返回0表示无错误）
    return 0;
}
