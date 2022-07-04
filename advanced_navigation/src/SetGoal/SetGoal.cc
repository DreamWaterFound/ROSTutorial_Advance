/**
 * @file SetGoal.cc
 * @author guoqing (1337841346@qq.com)
 * @brief 设置单次导航过程的路标点并获取导航状态
 * @version 0.1
 * @date 2020-06-03
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <string>
#include <sstream>
#include <iostream>

#include <ros/ros.h>
// TODO 补充消息头文件
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include<move_base_msgs/MoveBaseActionResult.h>
#include "std_srvs/Empty.h"
extern int arriveflag=1;
// ...

// 节点基类
#include "ExperNodeBase.hpp"

using namespace std;
/* ========================================== 宏定义 =========================================== */
#define MACRO_GOAL_POSE_TOPIC   "/move_base/goal"       // 发送导航目标点的 topic
#define MACRO_CLEAR_COST_MAP_SRV "/move_base/clear_costmaps"   // 清除 Costmap 的 service
// TODO 2.3.2 填写你选择的 topic
 #define MACRO_RESULT_TOPIC      "/move_base/result"             // 获取导航结果的 topic

#define CONST_PI                3.141592654f            // 圆周率

/* ========================================== 程序正文 =========================================== */

/** @brief 设置机器人导航路标点的节点 */
class SetGoalNode : public ExperNodeBase
{
public:
    /**
     * @brief 构造函数
     * @param[in] nArgc         命令行参数个数
     * @param[in] ppcArgv       命令行参数表列
     * @param[in] pcNodeName    当前节点运行时的名字
     */
    SetGoalNode(int nArgc, char** ppcArgv, const char* pcNodeName)
        : ExperNodeBase(nArgc, ppcArgv, pcNodeName)
    {

        // TODO 2.3.1 设置发布器
        mPubNextGoal = mupNodeHandle->advertise<move_base_msgs::MoveBaseActionGoal>(MACRO_GOAL_POSE_TOPIC, 1);
        // TODO 2.3.2 设置导航状态订阅器. 注意回调函数是类的成员函数, 或者是类的静态函数时, 后面应该怎么写; 方式不唯一
       mSubNavRes = mupNodeHandle->subscribe(MACRO_RESULT_TOPIC, 1, resCallback);
        //subscribe<move_base_msgs::MoveBaseActionResult>(MACRO_RESULT_TOPIC, 1,resCallback);
        // 确保初始化完成, 不然可能存在第一条消息发送不出去的情况
        ros::Duration(0.1).sleep();
    }

    /** @brief 析构函数 */
    ~SetGoalNode(){};

    /** @brief 主循环 */
    void Run(void) override
    {
        
        // TODO 2.3.1 发布导航目标点
        // 如果需要自己添加类成员变量或成员函数, 请随意添加
        // <YOUR CODE>

        double X,Y,Yaw;
        while(ros::ok())
        {
            if (arriveflag)
            {
                cout<<"please set your goal:"<<endl;
                cout<<"X:"<<endl;
                cin>>X;
                cout<<"Y:"<<endl;
                cin>>Y;
                cout<<"Yaw:"<<endl;
                cin>>Yaw;

            
                
                SetCurrGoal(X, Y, Yaw);

                while(!mPubNextGoal.getNumSubscribers())
                {
                    ros::Duration(0.1).sleep();
                }
            
              //ros::Duration(2).sleep();
                mPubNextGoal.publish(mMsgCurrGoal);
                cout<<"robot is achieving the goal..."<<endl;
                arriveflag=0;
                //接收订阅的消息
                ros::Rate loop_rate( 0.5);
                while(!arriveflag)
                {
                    ros::spinOnce();//ros消息回调处理函数，在调用后继续执行接下来的程序
                    //ROS_INFO("Wait for the accomplishment of the current goal.");
                    loop_rate.sleep();
                    //cout<<"arriveflag="<<arriveflag<<endl;
                }
            }
        }

    }

    // TODO 2.3.2 导航执行结果的回调函数
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
        arriveflag=1;
   }
 }

private:

    /**
     * @brief 根据传入坐标生成路径点
     * @param[in] dX            坐标X
     * @param[in] dY            坐标Y
     * @param[in] dYawDeg       偏航角, 角度制表示
     */
    void SetCurrGoal(double dX, double dY, double dYawDeg) //在终端输入x/y/yaw
    {
        auto& msgHeader         = mMsgCurrGoal.header;
        auto& msgGoalID         = mMsgCurrGoal.goal_id;
        auto& msgTargetHeader   = mMsgCurrGoal.goal.target_pose.header;
        auto& msgPt             = mMsgCurrGoal.goal.target_pose.pose.position;
        auto& msgQt             = mMsgCurrGoal.goal.target_pose.pose.orientation;

        // Step 1 初始化消息头
        int nCnt;
        msgHeader.seq= nCnt;
        msgHeader.stamp = ros::Time::now();
        msgHeader.frame_id   = "map";
        
        msgGoalID.stamp = msgHeader.stamp;
        msgTargetHeader = msgHeader;
         auto&timeStamp= msgHeader.stamp;
        /* NOTICE 这里的 id 设置有点意思, 有时间的同学可以进行下面的几个小测试:
         *      - 如果设置的id为空, 通过 rostopic echo 查看到发送的消息中, 这个字段是什么内容? 
         *      - 同样查看 rviz 发送的消息, 这个字段是什么内容? 多发送几次, 你能找到什么规律?
         *      - 将这里的id改为任何一个固定的非空字符串如"my_goal", 通过 rostopic echo 查看到这个字段是什么?
         *      - 修改坐标多发送几次呢?
         *      - 你发现 rviz 中机器人执行导航的过程的不正常情况了吗?
        */
        // std::stringstream ss;
        
        // ss << "my_goal_" << nCnt << "_" << timeStamp.sec << "." << timeStamp.nsec;
        // msgGoalID.id = ss.str().c_str();
        // ROS_INFO_STREAM("Goal Id: " << ss.str());//ROS_DEBUG_STREAM("Goal Id: " << ss.str());
   
        // Step 2 设置目标点
        // TODO 2.3.1 设置目标点
        msgPt .x=dX;
        msgPt.y=dY;
        msgPt.z=0;
        Heading2Quat(dYawDeg,msgQt );
        // cout<<"position set done!  "<<"   x=  "<<msgPt .x<<"   y=  "<<msgPt .y<<"   z=  "<<msgPt .z<<endl;
        // cout<<"orientation set done!"<<"   quat.x=  "<<msgQt .x<<"   quat.y=  "<<msgQt .y<<"quat.z=  "<<msgQt .z<<"quat.w=  "<<msgQt .w<<endl;
        // <YOUR CODE>
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
    
private:

    // TODO 2.3.2 补全类型
    ros::Publisher  mPubNextGoal;           // 路标点发布器
    ros::Subscriber mSubNavRes;             // 导航状态订阅器
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
    // 生成 ROS 节点对象
    SetGoalNode node(argc, argv, "set_goal");
    // 运行
    node.Run();
    return 0;
}

