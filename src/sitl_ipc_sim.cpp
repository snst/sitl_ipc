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
#include "sitl_ipc_sim.h"

#ifdef __cplusplus
extern "C"
{
#endif

    //    static pthread_t worker_thread_handle;
    static ros::NodeHandlePtr node;
    static struct sitl_motor_t motor_state;

    static ros::Publisher pub_sonar;
    static ros::Publisher pub_imu;
    static ros::Publisher pub_pos;
    static ros::Publisher pub_time;
    static ros::Publisher pub_gps;
    static ros::Subscriber sub_motor;
    static ros::Subscriber sub_resetworld;

    void sitl_set_gps(struct sitl_gps_t *data)
    {
        sensor_msgs::NavSatFix msg;
        msg.latitude = data->latitude;
        msg.longitude = data->longitude;
        msg.altitude = data->altitude;
        msg.status.status = data->status;
        pub_gps.publish(msg);
    }

    void sitl_set_pos(struct sitl_pos_t *data)
    {
        geometry_msgs::Vector3 msg;
        msg.x = data->x;
        msg.y = data->y;
        msg.z = data->z;
        pub_pos.publish(msg);
    }

    void sitl_set_imu(struct sitl_imu_t *data)
    {
        sensor_msgs::Imu msg;
        msg.linear_acceleration.x = data->linear_acceleration_x;
        msg.linear_acceleration.y = data->linear_acceleration_y;
        msg.linear_acceleration.z = data->linear_acceleration_z;

        msg.angular_velocity.x = data->angular_velocity_r;
        msg.angular_velocity.y = data->angular_velocity_p;
        msg.angular_velocity.z = data->angular_velocity_y;

        msg.orientation.w = data->orientation_quat_w;
        msg.orientation.x = data->orientation_quat_x;
        msg.orientation.y = data->orientation_quat_y;
        msg.orientation.z = data->orientation_quat_z;
        pub_imu.publish(msg);
    }

    void sitl_set_sonar(struct sitl_sonar_t *data)
    {
        sensor_msgs::Range msg;
        msg.range = data->distance;
        pub_sonar.publish(msg);
    }

    void sitl_set_simtime(double data)
    {
        std_msgs::Float64 msg;
        msg.data = data;
        pub_time.publish(msg);
    }
/*
    static sitl_motor_callback_t motor_callback = NULL;
    static void *motor_callback_param = NULL;

    static void update_motor(const sitl_ipc::MotorControl::ConstPtr &msg)
    {
        motor_state.motor[0] = msg->m0;
        motor_state.motor[1] = msg->m1;
        motor_state.motor[2] = msg->m2;
        motor_state.motor[3] = msg->m3;
        //        static int i = 0;
        //        i++;
        //        printf("update_motor %d", i++);
        if (motor_callback)
        {
            motor_callback(motor_callback_param, &motor_state);
        }
    }
*/
    boost::function<void(struct sitl_motor_t*)> motor_callback2;
    boost::function<void()> reset_world_callback;

    static void reset_world(const std_msgs::Bool::ConstPtr &msg)
    {
        printf("reset_world\n");
        if(reset_world_callback) {
            reset_world_callback();
        }
    }    

    static void update_motor2(const sitl_ipc::MotorControl::ConstPtr &msg)
    {
        motor_state.motor[0] = msg->m0;
        motor_state.motor[1] = msg->m1;
        motor_state.motor[2] = msg->m2;
        motor_state.motor[3] = msg->m3;
        //        static int i = 0;
        //        i++;
        //        printf("update_motor %d", i++);
        if (motor_callback2)
        {
            motor_callback2(&motor_state);
        }
    }


    void sitl_register_motor_callback2(boost::function<void(struct sitl_motor_t*)> cb)
    {
        motor_callback2 = cb;
        ros::SubscribeOptions jointStatesSo = ros::SubscribeOptions::create<sitl_ipc::MotorControl>(
            SITL_TOPIC_MOTOR, 1, update_motor2, ros::VoidPtr(), node->getCallbackQueue());
        jointStatesSo.transport_hints = ros::TransportHints().unreliable();
        sub_motor = node->subscribe(jointStatesSo);
    }

    void sitl_register_reset_world_callback(boost::function<void()> cb)
    {
        reset_world_callback = cb;
        ros::SubscribeOptions jointStatesSo = ros::SubscribeOptions::create<std_msgs::Bool>(
            SITL_TOPIC_RESET, 1, reset_world, ros::VoidPtr(), node->getCallbackQueue());
        jointStatesSo.transport_hints = ros::TransportHints().unreliable();
        sub_resetworld = node->subscribe(jointStatesSo);
    }
/*
    void sitl_register_motor_callback(void *param, sitl_motor_callback_t cb)
    {
        motor_callback = cb;
        motor_callback_param = param;

    }
*/
    void sitl_start_ipc()
    {
        printf("+sitl_start_ipc\n");

        node = boost::make_shared<ros::NodeHandle>();

        pub_sonar = node->advertise<sensor_msgs::Range>(SITL_TOPIC_SONAR, 100);
        pub_imu = node->advertise<sensor_msgs::Imu>(SITL_TOPIC_IMU, 100);
        pub_pos = node->advertise<geometry_msgs::Vector3>(SITL_TOPIC_POS, 100);
        pub_time = node->advertise<std_msgs::Float64>(SITL_TOPIC_TIME, 100);
        pub_gps = node->advertise<sensor_msgs::NavSatFix>(SITL_TOPIC_GPS, 100);
/*
        ros::SubscribeOptions jointStatesSo = ros::SubscribeOptions::create<sitl_ipc::MotorControl>(
            SITL_TOPIC_MOTOR, 1, update_motor, ros::VoidPtr(), node->getCallbackQueue());
        jointStatesSo.transport_hints = ros::TransportHints().unreliable();
        sub_motor = node->subscribe(jointStatesSo);
*/
        //pthread_create(&worker_thread_handle, NULL, worker_thread, NULL);
        printf("-sitl_start_ipc\n");
    }

    void sitl_stop_ipc()
    {
        printf("+sitl_stop_ipc\n");
        //run_worker_thread = false;
        //ros::shutdown();
        //pthread_join(worker_thread_handle, NULL);
        printf("-sitl_stop_ipc\n");
    }

#ifdef __cplusplus
}
#endif