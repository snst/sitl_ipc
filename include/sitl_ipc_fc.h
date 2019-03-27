#ifndef _SITL_IPC_FC_H_
#define _SITL_IPC_FC_H_

#include <stdint.h>
#include "sitl_ipc_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    struct sitl_sonar_t *sitl_get_sonar();
    struct sitl_gps_t *sitl_get_gps();
    struct sitl_imu_t *sitl_get_imu();
    struct sitl_pos_t *sitl_get_pos();
    double sitl_get_sim_time();

    typedef void (*sitl_state_callback_t)(struct sitl_state_t *state);

    void sitl_set_motor(struct sitl_motor_t* data);
    void sitl_reset_world();
    void sitl_register_state_callback(sitl_state_callback_t cb);
    void sitl_start_ipc();
    void sitl_stop_ipc();

#ifdef __cplusplus
}
#endif

#endif