#include "ros/ros.h"
#include "../include/serial_struct.h"
#include "../include/hardware/robot.h"
#include <iostream>
#include <thread>
#include <condition_variable>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_serial_port");
    ros::NodeHandle n;
    int f = 1000;
    ros::Rate r(f);
    robot rb;
    ROS_INFO("\033[1;32mSTART\033[0m");
    // ========================== singlethread send =====================
    // rb.test_ser_motor();
    // while (0)
    float Kp = 0.15;
    float Kd = 0.01;
    float target_pos = 0.0;
    float torque = 0.0;
    float F = 2.0;
    float T = 1/F;
    float w = 1.0 / T;
    float derta_t = 1.0 / f;
    int step = T / derta_t;
    int cont = 0;
    int dir = 1;

    float max = 0;
    float min = 2.0;
    float derta = (max - min) / step;
    float pos_start[10] = {-0.241, 0.24, 0.09, -0.3, -1.0, -0.2, 2.77, 0.76, -1.0, -2.6};
    float pos_end[10] =   {0.241, 1.01, -0.4, 0.1, -1.4, -0.6, 2.27, 1.26, -1.4, -3.0};
    float pos_derta[10];
    int dfasfasf = 0;
    ros::Rate aa(10);
    while (ros::ok() && dfasfasf<50)
    {
        for (size_t i = 0; i < 10; i++)
        {
            rb.Motors[i]->fresh_cmd((pos_start[i] + pos_end[i]) / 2, 0.0, 0.0, 1.0, 0.01);
        }
        rb.motor_send();
        dfasfasf++;
        aa.sleep();
    }
    while (ros::ok()) // 此用法为逐个电机发送控制指令
    {
        // ROS_INFO_STREAM("START");
        // cont += dir;
        // if (cont == step)
        // {
        //     dir *= -1;
        //     // cont = 0;
        // }
        // else if (cont == -1)
        // {
        //     dir *= -1;
        // }
        /////////////////////////send
        //////////test1////////////
        //////////固定位置控制///////
        {
            // for (motor *m : rb.Motors)
            // {
            //     torque = (target_pos-m->get_current_motor_state()->position)*Kp;
            //     m->fresh_cmd(0.0, 0.0, torque, 0.0, 0.0);
            // }
            // rb.motor_send();

            // target_pos -= derta*dir;
            // std::cout<<target_pos<<"  "<<cont<<std::endl;
            // torque = (target_pos - rb.Motors[6]->get_current_motor_state()->position) * Kp;
            // // std::cout<<torque<<std::endl;
            // rb.Motors[6]->fresh_cmd(target_pos, 0.0, torque, 0.0, 0.0);
            // rb.motor_send();
        }
        //////////test1////////////
        //////////周期位置控制///////
        cont++;
        for (size_t i = 0; i < 10; i++)
        {
            // target_pos[i] = pos_min[i] + cont * pos_derta[i];
            target_pos = (pos_end[i] + pos_start[i]) / 2 + (pos_start[i] - pos_end[i]) / 2 * sin(w * 2 * M_PIf32 * (cont * derta_t));
            std::cout << target_pos << " ";
            torque = (target_pos - rb.Motors[i]->get_current_motor_state()->position) * Kp + (0.0 - rb.Motors[i]->get_current_motor_state()->velocity) * Kd;
            rb.Motors[i]->fresh_cmd(target_pos, 0.0, torque, 0.0, 0.0);
        }
        std::cout << "\n";

        rb.motor_send();
        ////////////////////////recv
        for (motor *m : rb.Motors)
        {
            motor_back_t motor;
            motor = *m->get_current_motor_state();
            // ROS_INFO("ID:%d pos: %8f,vel: %8f,tor: %8f", motor.ID, motor.position, motor.velocity, motor.velocity);
        }
        // ROS_INFO_STREAM("END"); //
        r.sleep();
    }

    for (auto &thread : rb.ser_recv_threads)
    {
        thread.join();
    }
    ros::spin();
    return 0;
}
