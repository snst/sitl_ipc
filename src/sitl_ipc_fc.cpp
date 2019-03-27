#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sitl_ipc/MotorControl.h"
#include <stdint.h>
#include "sitl_ipc_fc.h"

#ifdef __cplusplus
extern "C"
{
#endif

    static struct sitl_state_t state;
    static pthread_t worker_thread_handle;
    static ros::NodeHandlePtr node;
    static bool run_worker_thread = false;
    static sitl_state_callback_t state_callback = NULL;
    static ros::Publisher motor_control_pub;
    static ros::Publisher reset_world_pub;

    void sitl_register_state_callback(sitl_state_callback_t cb)
    {
        //printf("sitl_register_state_callback %p\n", cb);
        state_callback = cb;
    }

    struct sitl_sonar_t *sitl_get_sonar()
    {
        return &state.sonar;
    }

    struct sitl_gps_t *sitl_get_gps()
    {
        return &state.gps;
    }

    struct sitl_imu_t *sitl_get_imu()
    {
        return &state.imu;
    }

    struct sitl_pos_t *sitl_get_pos()
    {
        return &state.pos;
    }

    double sitl_get_sim_time()
    {
        return state.sim_time;
    }

    static void update_gps(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        state.gps.latitude = msg->latitude;
        state.gps.longitude = msg->longitude;
        state.gps.satellites = 8;
        state.gps.status = 0;
    }

    static void update_sonar(const sensor_msgs::Range::ConstPtr &msg)
    {
        state.sonar.distance = (int32_t)(msg->range * 100);
    }

    static void update_imu(const sensor_msgs::Imu::ConstPtr &msg)
    {
        state.imu.linear_acceleration_x = msg->linear_acceleration.x;
        state.imu.linear_acceleration_y = msg->linear_acceleration.y;
        state.imu.linear_acceleration_z = msg->linear_acceleration.z;

        state.imu.angular_velocity_r = msg->angular_velocity.x;
        state.imu.angular_velocity_p = msg->angular_velocity.y;
        state.imu.angular_velocity_y = msg->angular_velocity.z;

        state.imu.orientation_quat_w = msg->orientation.w;
        state.imu.orientation_quat_x = msg->orientation.x;
        state.imu.orientation_quat_y = msg->orientation.y;
        state.imu.orientation_quat_z = msg->orientation.z;
    }

    static void update_pos(const geometry_msgs::Vector3::ConstPtr &msg)
    {
        state.pos.x = msg->x;
        state.pos.y = msg->y;
        state.pos.z = msg->z;
    }

    void update_time(const std_msgs::Float64::ConstPtr &msg)
    {
        state.sim_time = (double)msg->data;

        if (state_callback)
        {
            state_callback(&state);
        }
    }

    void sitl_set_motor(struct sitl_motor_t* data)
    {
        sitl_ipc::MotorControl msg;
        msg.m0 = data->motor[0];
        msg.m1 = data->motor[1];
        msg.m2 = data->motor[2];
        msg.m3 = data->motor[3];
        motor_control_pub.publish(msg);
    }

    void sitl_reset_world()
    {
        std_msgs::Bool msg;
        msg.data = true;
        reset_world_pub.publish(msg);
//        msg.data = false;
//        reset_world_pub.publish(msg);
    }

    static void *worker_thread(void *data)
    {
        printf("+worker_thread\n");
        run_worker_thread = true;

        //ros::Subscriber sub_joystick = node->subscribe("joy", 100, joystick_update);
        ros::Subscriber sub_sonar = node->subscribe(SITL_TOPIC_SONAR, 10, update_sonar);
        ros::Subscriber sub_imu = node->subscribe(SITL_TOPIC_IMU, 10, update_imu);
        ros::Subscriber sub_pos = node->subscribe(SITL_TOPIC_POS, 10, update_pos);
        ros::Subscriber sub_simtime = node->subscribe(SITL_TOPIC_TIME, 10, update_time);
        ros::Subscriber sub_gps = node->subscribe(SITL_TOPIC_GPS, 10, update_gps);

        while (run_worker_thread)
        {
            ros::spin();
        }
        printf("-worker_thread\n");
        return NULL;
    }

    void sitl_start_ipc()
    {
        printf("+sitl_start_ipc\n");

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "sitl_ipc_fc", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
        }

        node = boost::make_shared<ros::NodeHandle>();
        motor_control_pub = node->advertise<sitl_ipc::MotorControl>(SITL_TOPIC_MOTOR, 100);
        reset_world_pub = node->advertise<std_msgs::Bool>(SITL_TOPIC_RESET, 10);

        pthread_create(&worker_thread_handle, NULL, worker_thread, NULL);
        printf("-sitl_start_ipc\n");
    }

    void sitl_stop_ipc()
    {
        printf("+sitl_stop_ipc\n");
        run_worker_thread = false;
        ros::shutdown();
        pthread_join(worker_thread_handle, NULL);
        printf("-sitl_stop_ipc\n");
    }

#ifdef __cplusplus
}
#endif