#include "ros/ros.h"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <iostream>
#include <thread>
#include <condition_variable>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_motor_run");
    ros::NodeHandle n;
    ros::Rate r(1);
    livelybot_serial::robot rb;
    ROS_INFO("\033[1;32mSTART\033[0m");
    int cont = 0;
    size_t n_motors = 20;
    std::vector<std::string> motor_name{"null","5046","4538","5047_36","5047_9"}; 
    
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        // ROS_INFO_STREAM("START");
        /////////////////////////send
        for (size_t i = 0; i < n_motors; i++)
        {
            rb.Motors[i]->fresh_cmd(0.0, 0.0, 0.0, 10.0, 0.1);
        }

        rb.motor_send();
        ////////////////////////recv
        for (size_t i = 0; i < n_motors; i++)
         {
            motor_back_t motor;
            motor = *rb.Motors[i]->get_current_motor_state();
            ROS_INFO_STREAM("ID: " << motor.ID << 
                            " Pos: " << motor.position <<
                            " torque: " << motor.torque<<
                            "  type: " << motor_name[static_cast<int>(rb.Motors[i]->get_motor_enum_type())]
                            );
        }
        cont++;
        ROS_INFO("count:     %d",cont);
        if (cont==1800)
        {
            break;
        }
        r.sleep();
    }
    for (auto &thread : rb.ser_recv_threads)
    {
        thread.join();
    }
    return 0;
}
