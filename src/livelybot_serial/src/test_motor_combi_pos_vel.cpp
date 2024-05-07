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
    ros::Rate r(5);
    livelybot_serial::robot rb;
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    // rb.test_ser_motor();
    // while (0)
    int cont = 0;
    size_t n_motors = 6;
    double torque=0;
    double time=0;
    double pos=0,vel=0;
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        // if(torque>1)torque=0;
        torque+=0.01;
        time+=0.1;
        
        // ROS_INFO_STREAM("START");
        /////////////////////////send
        size_t i=5;

        pos=cos(time*M_PI);
        vel=-M_PI*sin(time*M_PI);

        rb.Motors[i]->fresh_cmd(pos,vel, 0, 0.5, 0.005);
        rb.motor_send();
        motor_back_t motor;
        motor = *rb.Motors[i]->get_current_motor_state();
        ROS_INFO_STREAM("ID: " << motor.ID << 
                            " desire pos: "<<pos<<
                            " get pos"<<  motor.position<<
                            " desire vel"<< vel<<
                            " get vel" <<motor.velocity<< 

                            " desire torque: " << torque <<
                            " get torque: " << motor.torque);
            
        
        // rb.Motors[9]->fresh_cmd(0.2, 0.0, 0.0, 1.5, 0.01);
        ////////////////////////recv
        cont++;
        if (cont==180000)
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
