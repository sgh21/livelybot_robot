#include "../../include/hardware/motor.h"
template <typename T>
inline T motor::float2int(float in_data, uint8_t type)
{
    switch (type)
    {
    case 0: // radian float pos/vel to int32
        return (int32_t)(in_data / my_2pi * 100000.0);
    case 1: // angle float pos/vel to int32
        return (int32_t)(in_data / 360.0 * 100000.0);
    case 2: // float kp/kd to int16 
        return (int16_t)(in_data * 0x7FF);
    case 3: // float torque to int32 5046
        return (int32_t)((in_data )/ 0.000528);
    case 4: //float torque to int32 4538
        return (int32_t)((in_data )/0.000445);
    case 5: //float torque to int32 5047 36 减速比
        return (int32_t)((in_data )/ 0.000462);
    case 6: //float torque to int32 5047 36 减速比
        return (int32_t)((in_data )/ 0.000533);
    default:
        return T();
    }
}

inline float motor::int2float(int32_t in_data, uint8_t type)
{
    switch (type)
    {
    case 0: // radian float pos/vel to int32
        return (float)(in_data * my_2pi / 100000.0);
    case 1: // angle float pos/vel to int32
        return (float)(in_data * 360.0 / 100000.0);
    case 2: // float torque to int32 5046
        return (float)(in_data *0.000528);
    case 3: // float torque to int32 4538
        return (float)(in_data *0.000445);
    case 4: // float torque to int32 5047 36减速比
        return (float)(in_data *0.000462);
    case 5: // float torque to int32 5047 9减速比
        return (float)(in_data *0.000533);
    default:
        return float();
    }
}

void motor::fresh_cmd(float position, float velocity, float torque, float Kp, float Kd)
{
    switch(type_)
    {
        case motor_type::null:
            ROS_INFO("error,motor type not set,fresh command fault");
            break;
        case motor_type::_5046:
            cmd.motor_cmd.position = float2int<int32_t>(position, 0);
            cmd.motor_cmd.velocity = float2int<int32_t>(velocity, 0);
            cmd.motor_cmd.torque = float2int<int32_t>(torque, 3);
            cmd.motor_cmd.Kp = float2int<int16_t>(Kp, 2);
            cmd.motor_cmd.Kd = float2int<int16_t>(Kd, 2);
            cmd.crc16 = crc_ccitt(0x0000, (const uint8_t *)&cmd, sizeof(cdc_acm_rx_message_t) - 2);
            break;
        case motor_type::_4538:
            cmd.motor_cmd.position = float2int<int32_t>(position, 0);
            cmd.motor_cmd.velocity = float2int<int32_t>(velocity, 0);
            cmd.motor_cmd.torque = float2int<int32_t>(torque, 4);
            cmd.motor_cmd.Kp = float2int<int16_t>(Kp, 2);
            cmd.motor_cmd.Kd = float2int<int16_t>(Kd, 2);
            cmd.crc16 = crc_ccitt(0x0000, (const uint8_t *)&cmd, sizeof(cdc_acm_rx_message_t) - 2);
            break;
        case motor_type::_5047_36:
            cmd.motor_cmd.position = float2int<int32_t>(position, 0);
            cmd.motor_cmd.velocity = float2int<int32_t>(velocity, 0);
            cmd.motor_cmd.torque = float2int<int32_t>(torque, 5);
            cmd.motor_cmd.Kp = float2int<int16_t>(Kp, 2);
            cmd.motor_cmd.Kd = float2int<int16_t>(Kd, 2);
            cmd.crc16 = crc_ccitt(0x0000, (const uint8_t *)&cmd, sizeof(cdc_acm_rx_message_t) - 2);
            break;
        case motor_type::_5047_9:
            cmd.motor_cmd.position = float2int<int32_t>(position, 0);
            cmd.motor_cmd.velocity = float2int<int32_t>(velocity, 0);
            cmd.motor_cmd.torque = float2int<int32_t>(torque, 6);
            cmd.motor_cmd.Kp = float2int<int16_t>(Kp, 2);
            cmd.motor_cmd.Kd = float2int<int16_t>(Kd, 2);
            cmd.crc16 = crc_ccitt(0x0000, (const uint8_t *)&cmd, sizeof(cdc_acm_rx_message_t) - 2);
            break;
        default:
            ROS_INFO("motor type setting error");
            return;
    }

    // ROS_INFO("CRC: 0x%x",cmd.crc16);
}

void motor::fresh_data(int32_t position, int32_t velocity, int32_t torque)
{
    switch(type_)
    {
        case motor_type::null:
            ROS_INFO("error,motor type not set,fresh data fault");
            break;
        case motor_type::_5046:
            p_msg.pos = data.position = int2float(position, 0);
            p_msg.vel = data.velocity = int2float(velocity, 0);
            p_msg.tau = data.torque = int2float(torque, 2);
            break;
        case motor_type::_4538:
            p_msg.pos = data.position = int2float(position, 0);
            p_msg.vel = data.velocity = int2float(velocity, 0);
            p_msg.tau = data.torque = int2float(torque, 3);
            break;
        case motor_type::_5047_36:
            p_msg.pos = data.position = int2float(position, 0);
            p_msg.vel = data.velocity = int2float(velocity, 0);
            p_msg.tau = data.torque = int2float(torque, 4);
            break;
        case motor_type::_5047_9:
            p_msg.pos = data.position = int2float(position, 0);
            p_msg.vel = data.velocity = int2float(velocity, 0);
            p_msg.tau = data.torque = int2float(torque, 5);
            break;
        default:
            ROS_INFO("motor type setting error");
            return;
    }

    _motor_pub.publish(p_msg);
}