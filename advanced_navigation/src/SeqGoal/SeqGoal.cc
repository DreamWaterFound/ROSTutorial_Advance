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
// TODO 补充头文件
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include<move_base_msgs/MoveBaseActionResult.h>
#include "std_srvs/Empty.h"

// 节点基类
#include "ExperNodeBase.hpp"

/* ========================================== 宏定义 =========================================== */
#define MACRO_GOAL_POSE_TOPIC       "/move_base/goal"       // 发送导航目标点的 topic
#define MACRO_RESULT_TOPIC          "/move_base/result"        // 获取导航结果的 topic

#define CONST_PI                    3.141592654f            // 圆周率
using namespace std;

/* ========================================== 全局变量 =========================================== */
bool gbQuit = false;
extern int arriveflag=1;
extern long int line=0;
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

/** @brief 设置机器人导航路标点序列的节点 */
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

        // TODO 完成设置
        mPubNextGoal = mupNodeHandle->advertise<move_base_msgs::MoveBaseActionGoal>(MACRO_GOAL_POSE_TOPIC, 1);
        mSubNavRes   = mupNodeHandle->subscribe(MACRO_RESULT_TOPIC, 1, resCallback);
        //mSubSigRes   = mupNodeHandle->subscribe(MACRO_RESULT_TOPIC, 100, OnSignalInterrupt);

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
        sa.sa_handler = OnSignalInterrupt;
         assert(sigaction(SIGINT,&sa,NULL)!=-1);//捕获到信号sig，使用sa中规定的方法处理
        while(!mPubNextGoal.getNumSubscribers())
        {
            ros::Duration(0.1).sleep();
        }

        // Step 1 从外部文件中读取数据
        if(mifsLandMarks.is_open() == false)
        {
            ROS_FATAL_STREAM("Can not open landmarks file \"" << mstrLandmarksFile << "\"!");
            return ;
        }
        else
        {
            ROS_INFO_STREAM("Landmarks file \"" << mstrLandmarksFile << "\" opened.");
            // 预先读取前三行的注释
            std::string strDummy;
            getline(mifsLandMarks,strDummy);
            getline(mifsLandMarks,strDummy);
            getline(mifsLandMarks,strDummy);
        }
        // 读取点个数
        ReadLandMarkNum();
        cout<<"We have " << mnMaxLandMarkId <<" landmark(s)."<<endl;

        // 处理每一个路标点
        
        for(size_t nId = 0; 
            nId < mnMaxLandMarkId && ros::ok() && !gbQuit; // && /* YOUR CONDITION */
            ++nId)
        {
              double  X,Y,Yaw=0;
              ReadNextLandMark(X, Y, Yaw);
              cout<<"line_"<<line<<":  x="<<X<<"  y="<<Y<<"  yaw="<<Yaw<<endl;

               SetCurrGoal(X, Y, Yaw);

                mPubNextGoal.publish(mMsgCurrGoal);
                cout<<"x="<<mMsgCurrGoal.goal.target_pose.pose.position.x<<"  y="<<mMsgCurrGoal.goal.target_pose.pose.position.y<<endl;
                cout<<"robot is achieving the goal..."<<endl;
                arriveflag=0;
                ros::Rate loop_rate( 0.5);
                while(!arriveflag)
                {
                    ros::spinOnce();//ros消息回调处理函数，在调用后继续执行接下来的程序
                    //ROS_INFO("Wait for the accomplishment of the current goal.");
                    loop_rate.sleep();
                }
                   // 使用这个实现延时2秒
                    line++;
             
              ros::Duration(2).sleep();
        }
        if(line==mnMaxLandMarkId)
        {
        cout<<"Congradulations!!  Robot has already passed"<< mnMaxLandMarkId <<" landmark(s)."<<endl;
        }
    }

    // TODO 增加你的成员函数
static  void resCallback(const move_base_msgs::MoveBaseActionResult& msg) // 回调函数
 {
   if(msg.status.status==msg.status.SUCCEEDED)
   {
       cout<<"SUCCEED! Robot reachs the goal."<<endl;
       arriveflag=1;

   }
   else if(msg.status.status==msg.status.ABORTED)
   {
        cout<<"ERROR:ABORTED."<<endl;
   }
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
        int nCnt;
        ros::Time timeStamp = ros::Time::now();

        // Step 1 初始化消息头
        msgHeader.seq = nCnt;
        msgHeader.stamp = ros::Time::now();
        msgHeader.frame_id   = "map";
        
        msgGoalID.stamp = msgHeader.stamp;
        msgTargetHeader = msgHeader;

        // std::stringstream ss;
        // ss << "my_goal_" << nCnt << "_" << timeStamp.sec << "." << timeStamp.nsec;
        // msgGoalID.id = ss.str().c_str();
       // ROS_INFO_STREAM("Goal Id: " << ss.str());

        // TODO 设置坐标
        // <YOUR CODE>
        msgPt .x=dX;
        msgPt.y=dY;
        msgPt.z=0;
        Heading2Quat(dYawDeg,msgQt );
        
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

    struct sigaction sa;
    ros::Publisher  mPubNextGoal;           ///< 发布器, 发布下一个路标点
    ros::Subscriber mSubNavRes;             ///< 订阅器, 获取当前导航状态

    std::ifstream   mifsLandMarks;          ///< 外部路标点文件的输入流对象
    std::string     mstrLandmarksFile;      ///< 外部路标点文件
    size_t          mnMaxLandMarkId;        ///< 外部路标点文件中的总点数
// TODO 补充你自己的成员变量(如果有的话)
    move_base_msgs::MoveBaseActionGoal mMsgCurrGoal;
    
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

