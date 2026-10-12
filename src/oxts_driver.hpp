/**************************************************************************
Copyright 2026 - Dataspeed Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
**************************************************************************/

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <netinet/in.h>
#include "NComRxC.h"
#include "nav_const.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <nmea_msgs/msg/sentence.hpp>
#include <rtcm_msgs/msg/message.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include "kinematics_transformer.hpp"
#include <gps_tools/conversions.h>

#include <eigen3/Eigen/Dense>

namespace oxts_ros2
{

    class UdpClient
    {
    public:
        UdpClient(const std::string &host, int port);
        ~UdpClient();
        bool initialize_socket(const bool = false);
        void close_socket();
        bool is_initialized() const;
        ssize_t read(void *buffer, size_t count);
        ssize_t send(const void *buffer, size_t count);

    private:
        int socket_fd_ = -1;
        bool initialized_ = false;
        std::string host_;
        int port_;
        struct sockaddr_in server_addr_;
    };

    class OxtsDriver : public rclcpp::Node
    {
    public:
        OxtsDriver(const rclcpp::NodeOptions &options);
        ~OxtsDriver() override;

    private:
        // Configuration parameters
        std::string udp_host_;
        int udp_port_;
        std::string broadcast_host_;
        int broadcast_port_;
        double ncom_rate_;
        double gga_period_;
        std::string imu_frame_id_;
        std::string base_frame_;
        bool publish_tf_;
        double mount_rotation_tolerance_deg_;

        // UDP client and receiver thread
        std::unique_ptr<UdpClient> udp_client_;
        std::unique_ptr<UdpClient> broadcast_client_;
        std::thread receiver_thread_;
        std::atomic<bool> running_;

        // Data buffer and mutex for thread safety
        static constexpr size_t BUFFER_SIZE = 4096;
        std::array<uint8_t, BUFFER_SIZE> data_buffer_;
        std::mutex data_mutex_;

        // NCOM decoding
        NComRxC *nrx;
        double last_week_second_ = 0.0;
        bool check_rate(double previous_timestamp, double current_timestamp);
        sensor_msgs::msg::Imu populate_imu(const NComRxC *nrx, std_msgs::msg::Header header);
        sensor_msgs::msg::NavSatFix populate_fix(const NComRxC *nrx, std_msgs::msg::Header header);
        geometry_msgs::msg::TwistStamped populate_velocity(const NComRxC *nrx, std_msgs::msg::Header header);
        geometry_msgs::msg::Vector3 populate_angular_acceleration(const NComRxC *nrx);
        sensor_msgs::msg::NavSatStatus populate_fix_status(const NComRxC *nrx);
        sensor_msgs::msg::TimeReference populate_time_reference(const NComRxC *nrx, std_msgs::msg::Header header);

        // ROS Publishers
        void publish_ros_msgs();
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
        rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_reference_pub_;
        rclcpp::Publisher<nmea_msgs::msg::Sentence>::SharedPtr gga_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        int gga_downsample_counter_ = 0;
        int gga_counter_rollover_ = 200;

        // ROS subscribers
        rclcpp::Subscription<rtcm_msgs::msg::Message>::SharedPtr rtcm_sub_;
        void rtcm_callback(const rtcm_msgs::msg::Message::ConstSharedPtr msg);
        void generate_gga_string(double latitude, double longitude, double altitude, const rclcpp::Time &utc_time, std::string &gga_msg);

        // Transforms
        tf2::Quaternion getVat();
        tf2::Quaternion getVehRPY(const NComRxC *nrx);
        bool imu2veh_logged_ = false;
        void log_imu2veh(const NComRxC *nrx);

        // Lever arm compensation
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
        bool rviz_frame_published_ = false;
        std::string utm_zone_;
        KinematicsTransformer::SharedPtr kinematics_transformer_;

        bool init_kinematics_transformer(const std::string &ins_frame);
        bool process_kinematics(sensor_msgs::msg::Imu &imu,
                                geometry_msgs::msg::TwistStamped &vel,
                                const geometry_msgs::msg::Vector3 &ins_angular_acceleration,
                                sensor_msgs::msg::NavSatFix &fix,
                                nav_msgs::msg::Odometry &odom);

        /**
         * @brief Meridian convergence of the UTM grid at a point
         *
         * Angle from true north to grid north, positive counter-clockwise about up. A yaw in ENU is
         * converted to a yaw in the UTM grid by adding this angle.
         *
         * @param lat_deg Latitude in degrees
         * @param lon_deg Longitude in degrees
         * @param zone_number UTM zone number the point was projected into
         * @return Convergence angle in radians
         */
        static double utm_grid_convergence(double lat_deg, double lon_deg, int zone_number);

        // Thread worker function
        void receiver_worker();

        // Process received data
        void process_data(const std::array<uint8_t, BUFFER_SIZE> &data, size_t bytes_read);
    };
}
