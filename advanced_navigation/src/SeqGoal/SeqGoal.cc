/**
 * @file SeqGoal.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 从文件获取并设置一系列的的机器人位姿, 机器人逐步走过所有路径点
 * @version 0.1
 * @date 2020-06-03
 * 
 * @copyright Copyright (c) 2020
 * 
 */

/* ========================================== 头文件 =========================================== */
// C++ STL
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>

// Linux Signal
#include <signal.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>


// 节点基类
#include "ExperNodeBase.hpp"

/* ========================================== 宏定义 =========================================== */
#define MACRO_GOAL_POSE_TOPIC       "/move_base/goal"       // 发送导航目标点的 topic
#define MACRO_RESULT_TOPIC          "/move_base/result"        // 获取导航结果的 topic

#define CONST_PI                    3.141592654f            // 圆周率
using namespace std;

/* ========================================== 全局变量 =========================================== */
bool gbQuit = false;
int listen_count=0;
int listen_goal=0;

/* ========================================== 程序正文 =========================================== */

/**
 * @brief Linux 信号回调函数
 * @details 用于接收 Ctrl+C 产生的 SIG_INT 信号, 避免 ROS 节点运行时按 Ctrl+C 无法退出的问题
 * @param[in] nSigId 信号id
 */
void OnSignalInterrupt(int nSigId)
{
    std::cout << "Ctrl+C Pressed, program terminated." << std::endl;
    gbQuit = true;
}

/** @brief 设置机器人导航路标点序列 */
class SeqGoalNode : public ExperNodeBase
{
public:
    /**
     * @brief 构造函数
     * @param[in] nArgc         命令行参数个数
     * @param[in] ppcArgv       命令行参数表列
     * @param[in] pcNodeName    当前节点运行时的名字
     */
    SeqGoalNode(int nArgc, char** ppcArgv, const char* pcNodeName)
        : ExperNodeBase(nArgc, ppcArgv, pcNodeName)
    {

        // TODO
        //设置发布器
        mPubNextGoal = mupNodeHandle->advertise<move_base_msgs::MoveBaseActionGoal>(MACRO_GOAL_POSE_TOPIC, 1);
        mSubNavRes = mupNodeHandle->subscribe<move_base_msgs::MoveBaseActionResult>(MACRO_RESULT_TOPIC, 10, status_callback);
        
        // 打开文件
        mifsLandMarks.open(ppcArgv[1]);
        mstrLandmarksFile = std::string(ppcArgv[1]);
        
        // 确保初始化完成, 不然可能存在第一条消息发送不出去的情况
        ros::Duration(0.1).sleep();
    }

    /** @brief 析构函数 */
    ~SeqGoalNode()
    {   
        // 记得关闭打开的文件
        if(mifsLandMarks.is_open())
        {
            mifsLandMarks.close();
        }
    };

    /** @brief 主循环 */
    void Run(void) override
    {

        double dX=0;
        double dY=0;
        double dYawDeg=0;
        
        // Step 1 从外部文件中读取数据
        if(mifsLandMarks.is_open() == false)
        {
            ROS_FATAL_STREAM("Can not open landmarks file \"" << mstrLandmarksFile << "\"!");
            return ;
        }
        else
        {
            ROS_INFO_STREAM("Landmarks file \"" << mstrLandmarksFile << "\" opened.");
            // 预先读取前三行注释
            std::string strDummy;
            getline(mifsLandMarks,strDummy);
            getline(mifsLandMarks,strDummy);
            getline(mifsLandMarks,strDummy);
        }
        // 读取点个数
        ReadLandMarkNum();

        // 处理每一个路标点
        for(size_t nId = 0; 
            nId < mnMaxLandMarkId && ros::ok() && !gbQuit; // && /* YOUR CONDITION */
            ++nId)
        {
            //读取下一个导航目标点
            ReadNextLandMark(dX, dY, dYawDeg);
            //发布目标导航点
            SetCurrGoal(dX, dY, dYawDeg);
            mPubNextGoal.publish(mMsgCurrGoal);
            ROS_INFO("Navigation Goal Is Published!");

            ros::Rate loop_rate(5);
            // 回调函数未运行就一直等待，回调运行一次就继续
            listen_goal++;
            while(ros::ok() && listen_count<listen_goal)
            {
                ros::spinOnce();
                loop_rate.sleep();
                cout<<listen_count<<" "<<listen_goal<<endl;
            }

            cout<<"Please Wait For 2 Sec."<<endl;
            //延时2秒
            ros::Duration(2).sleep();
        }

        // TODO 
        // <YOUR CODE>
    }

    // TODO 增加你的成员函数
    //导航执行结果的回调函数
    static void status_callback(const move_base_msgs::MoveBaseActionResult msg){	
        if(msg.status.status == 3)
        {
            cout<<"Goal Was Achieved Successfully!"<<endl;
        }
        if(msg.status.status == 4)
        {
            cout<<"Goal Was Aborted!"<<endl;
        }
        listen_count++;
    }

private:

    /**
     * @brief 根据传入坐标生成路径点
     * @param[in] dX            坐标X
     * @param[in] dY            坐标Y
     * @param[in] dYawDeg       偏航角, 角度制表示
     */
    void SetCurrGoal(double dX, double dY, double dYawDeg)
    {
        auto& msgHeader         = mMsgCurrGoal.header;
        auto& msgGoalID         = mMsgCurrGoal.goal_id;
        auto& msgTargetHeader   = mMsgCurrGoal.goal.target_pose.header;
        auto& msgPt             = mMsgCurrGoal.goal.target_pose.pose.position;
        auto& msgQt             = mMsgCurrGoal.goal.target_pose.pose.orientation;

        ros::Time timeStamp = ros::Time::now();

        // Step 1 初始化消息头
        static int nCnt=0;
        nCnt++;
        msgHeader.seq = nCnt;
        msgHeader.stamp = ros::Time::now();
        msgHeader.frame_id   = "map";
        
        msgGoalID.stamp = msgHeader.stamp;
        msgTargetHeader = msgHeader;

        std::stringstream ss;
        ss << "my_goal_" << nCnt << "_" << timeStamp.sec << "." << timeStamp.nsec;
        msgGoalID.id = ss.str().c_str();
        ROS_INFO_STREAM("Goal Id: " << ss.str());

        // TODO 设置坐标
        msgPt.x=dX;
        msgPt.y=dY;
        msgPt.z=0;
        Heading2Quat(dYawDeg, msgQt);
        
    }

     /**
     * @brief 计算偏航到四元数的转换
     * @param[in]  dYawDeg  偏航角, 角度表示
     * @param[out] quat     转换后的四元数
     */
    void Heading2Quat(double dYawDeg, geometry_msgs::Quaternion& quat)
    {
        // 实现偏航向四元数的换算
        double dFaiDiv2_rad = 0.5f * dYawDeg / CONST_PI;
        quat.x = 0.0f;
        quat.y = 0.0f;
        quat.z = sin(dFaiDiv2_rad);
        quat.w = cos(dFaiDiv2_rad);
    }
    
    /** @brief 从文件中读取路径点的数目 */
    void ReadLandMarkNum(void)
    {
        std::string strDummy;
        std::stringstream ss;
        getline(mifsLandMarks,strDummy);
        ss << strDummy;
        ss >> mnMaxLandMarkId;
        ROS_INFO_STREAM("We have " << mnMaxLandMarkId << " landmark(s).");
    }

    /**
     * @brief 从文件中读取下一个路径点
     * @param[out] dX       X 坐标
     * @param[out] dY       Y 坐标
     * @param[out] dYAW     偏航
     */
    void ReadNextLandMark(double& dX, double& dY, double& dYAW)
    {
        std::string strDummy;
        std::stringstream ss;
        getline(mifsLandMarks,strDummy);
        ss << strDummy;
        ss >> dX >> dY >> dYAW;
    }

    

private:

    ros::Publisher  mPubNextGoal;           ///< 发布器, 发布下一个路标点
    ros::Subscriber mSubNavRes;             ///< 订阅器, 获取当前导航状态

    std::ifstream   mifsLandMarks;          ///< 外部路标点文件的输入流对象
    std::string     mstrLandmarksFile;      ///< 外部路标点文件
    size_t          mnMaxLandMarkId;        ///< 外部路标点文件中的总点数

    move_base_msgs::MoveBaseActionGoal  mMsgCurrGoal; //导航目标点信息
    
};


/**
 * @brief 主函数
 * @param[in] argc 命令行参数个数
 * @param[in] argv 命令行参数表列
 * @return int     执行返回码
 */
int main(int argc, char **argv)
{
    // 对于按照位置解析的参数, 必须要这样写, 不然的话 roslaunch 运行的时候会额外送很多参数, 这里会认为给的参数不正确
    if(argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " landmarks_file" << std::endl;
        return 0;
    }

    // 生成节点
    SeqGoalNode node(argc, argv, "seq_goal");
    // 运行
    node.Run();
    return 0;
}

