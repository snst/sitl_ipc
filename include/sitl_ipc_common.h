#ifndef _SITL_IPC_COMMON_H_
#define _SITL_IPC_COMMON_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SITL_TOPIC_SONAR "sitl_sonar"
#define SITL_TOPIC_GPS   "sitl_gps"
#define SITL_TOPIC_IMU   "sitl_imu"
#define SITL_TOPIC_POS   "sitl_pos"
#define SITL_TOPIC_TIME  "sitl_time"
#define SITL_TOPIC_MOTOR "sitl_motor"
#define SITL_TOPIC_AUX   "sitl_aux"
#define SITL_TOPIC_RESET "sitl_reset"

#define MAX_MOTOR 4

struct sitl_imu_t
{
    double linear_acceleration_x;
    double linear_acceleration_y;
    double linear_acceleration_z;

    double angular_velocity_r;
    double angular_velocity_p;
    double angular_velocity_y;

    double orientation_quat_w;
    double orientation_quat_x;
    double orientation_quat_y;
    double orientation_quat_z;
};

struct sitl_sonar_t
{
    int32_t distance;
};

struct sitl_gps_t
{
    double longitude;
    double latitude;
    double altitude;
    int8_t status;
    uint8_t satellites;
};

struct sitl_pos_t
{
    double x;
    double y;
    double z;
};


struct sitl_motor_t 
{
    float motor[MAX_MOTOR];
};

struct sitl_state_t
{
    double sim_time;
    struct sitl_imu_t imu;
    struct sitl_sonar_t sonar;
    struct sitl_gps_t gps;
    struct sitl_pos_t pos;
};


#ifdef __cplusplus
}
#endif

#endif