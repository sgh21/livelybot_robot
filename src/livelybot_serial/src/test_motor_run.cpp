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
    robot rb;
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    // rb.test_ser_motor();
    // while (0)
    int cont = 0;
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        // ROS_INFO_STREAM("START");
        /////////////////////////send
        for (motor *m : rb.Motors)
        {
            m->fresh_cmd(0.0, 0.0, 0.0, 1.0, 0.0);
        }
        rb.motor_send();
        ////////////////////////recv
        for (motor *m : rb.Motors)
        {
            motor_back_t motor;
            motor = *m->get_current_motor_state();
        }
        cont++;
        ROS_INFO("%d",cont);
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
