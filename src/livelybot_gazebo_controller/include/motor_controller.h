#ifndef _MOTOR_CONTROLLER_H_
#define _MOTOR_CONTROLLER_H_

#include <ros/node_handle.h>
#include <urdf/model.h>
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>


#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>

#include "livelybot_msg/MotorCmd.h"
#include "livelybot_msg/MotorState.h"
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>

#include "livelybot_gazebo_motor_control_tool.h"

namespace livelybot_gazebo_motor_control
{
    class LivelyBotGazeboMotorController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    private:
        hardware_interface::JointHandle joint;
        ros::Subscriber sub_cmd, sub_ft;
        control_toolbox::Pid pid_controller_;
        boost::scoped_ptr<realtime_tools::RealtimePublisher<livelybot_msg::MotorState>> controller_state_publisher_;
        const std::string& name="HuangPX-LD";
    public:
        std::string name_space;
        std::string joint_name;
        float sensor_torque;
        bool isHip, isThigh, isCalf, rqtTune;
        urdf::JointConstSharedPtr joint_urdf;
        realtime_tools::RealtimeBuffer<livelybot_msg::MotorCmd> command;
        livelybot_msg::MotorCmd lastCmd;
        livelybot_msg::MotorState lastState;
        ServoCmd servoCmd;

        LivelyBotGazeboMotorController(/* args */);
        ~LivelyBotGazeboMotorController();
        virtual bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);
        virtual void starting(const ros::Time &time);
        virtual void update(const ros::Time &time, const ros::Duration &period);
        virtual void stopping();
        void setTorqueCB(const geometry_msgs::WrenchStampedConstPtr &msg);
        void setCommandCB(const livelybot_msg::MotorCmdConstPtr &msg);
        void positionLimits(double &position);
        void velocityLimits(double &velocity);
        void effortLimits(double &effort);

        void setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min, const bool &antiwindup = false);
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
        void getGains(double &p, double &i, double &d, double &i_max, double &i_min);

    };    
}
#endif