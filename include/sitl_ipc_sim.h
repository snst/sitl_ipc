#ifndef _SITL_IPC_SIM_H_
#define _SITL_IPC_SIM_H_

#include <stdint.h>
#include "sitl_ipc_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    //typedef void (*sitl_motor_callback_t)(void *param, struct sitl_motor_t *motor);
    //void sitl_register_motor_callback(void *param, sitl_motor_callback_t cb);
    void sitl_register_motor_callback2(boost::function<void(struct sitl_motor_t *)> cb);
    void sitl_register_reset_world_callback(boost::function<void()> cb);

    void sitl_set_gps(struct sitl_gps_t *data);
    void sitl_set_pos(struct sitl_pos_t *data);
    void sitl_set_imu(struct sitl_imu_t *data);
    void sitl_set_sonar(struct sitl_sonar_t *data);
    void sitl_set_simtime(double data);

    void sitl_start_ipc();
    void sitl_stop_ipc();

#ifdef __cplusplus
}
#endif

#endif