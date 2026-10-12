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

#include <cfloat>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#include "oxts_driver.hpp"

namespace oxts_ros2
{
    namespace
    {
        geometry_msgs::msg::Vector3 to_vector3_msg(const Eigen::Vector3d &v)
        {
            geometry_msgs::msg::Vector3 msg;
            msg.x = v.x();
            msg.y = v.y();
            msg.z = v.z();
            return msg;
        }

        geometry_msgs::msg::Quaternion to_quaternion_msg(const Eigen::Quaterniond &q)
        {
            geometry_msgs::msg::Quaternion msg;
            msg.w = q.w();
            msg.x = q.x();
            msg.y = q.y();
            msg.z = q.z();
            return msg;
        }
    }

    OxtsDriver::OxtsDriver(const rclcpp::NodeOptions &options) : rclcpp::Node("oxts_driver", options),
                                                                 running_(false)
    {
        // Declare and get parameters
        udp_host_ = this->declare_parameter<std::string>("unit_ip", "192.168.1.3");
        udp_port_ = this->declare_parameter<int>("unit_port", 3000);
        broadcast_host_ = this->declare_parameter<std::string>("broadcast_ip", "255.255.255.255");
        broadcast_port_ = this->declare_parameter<int>("broadcast_port", 50472);
        ncom_rate_ = this->declare_parameter<double>("ncom_rate", 100.0);
        gga_period_ = this->declare_parameter<double>("gga_period", 2.0);
        imu_frame_id_ = this->declare_parameter<std::string>("imu_frame_id", "imu");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_footprint");
        publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
        mount_rotation_tolerance_deg_ = this->declare_parameter<double>("mount_rotation_tolerance_deg", 0.1);

        if (imu_frame_id_.empty())
        {
            RCLCPP_WARN(get_logger(), "imu_frame_id is empty, using 'imu' as default");
            imu_frame_id_ = "imu";
        }

        if (ncom_rate_ < 1.0 || gga_period_ < 0.1)
        {
            RCLCPP_WARN_STREAM(get_logger(), "Bad settings for ncom_rate and/or gga_period:  ncom_rate = " << this->ncom_rate_ << "  gga_period = " << this->gga_period_);
            RCLCPP_WARN(get_logger(), "Defaulting to ncom_rate = 100 and gga_period = 2");
            this->ncom_rate_ = 100.0;
            this->gga_counter_rollover_ = 200;
        }
        else
        {
            this->gga_counter_rollover_ = (int)(ncom_rate_ * gga_period_);
        }

        RCLCPP_INFO(this->get_logger(), "base_frame: %s", base_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "publish_tf: %s", publish_tf_ ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "mount_rotation_tolerance_deg: %f", mount_rotation_tolerance_deg_);

        this->nrx = NComCreateNComRxC();

        // TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        if (publish_tf_)
        {
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
            static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        }

        // Initialize ROS publishers
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("fix", 10);
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 10);
        time_reference_pub_ = this->create_publisher<sensor_msgs::msg::TimeReference>("time_reference", 10);
        gga_pub_ = this->create_publisher<nmea_msgs::msg::Sentence>("nmea", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

        // Initialize ROS subscribers
        rtcm_sub_ = this->create_subscription<rtcm_msgs::msg::Message>("rtcm", 10, std::bind(&OxtsDriver::rtcm_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Initializing OXTS driver with UDP: %s:%d",
                    udp_host_.c_str(), udp_port_);

        // Create UDP client
        udp_client_ = std::make_unique<UdpClient>(udp_host_, udp_port_);
        broadcast_client_ = std::make_unique<UdpClient>(broadcast_host_, broadcast_port_);

        // Start receiver thread last so everything it touches is already initialized
        running_ = true;
        receiver_thread_ = std::thread(&OxtsDriver::receiver_worker, this);

        RCLCPP_INFO(this->get_logger(), "OXTS driver initialized successfully");
    }

    OxtsDriver::~OxtsDriver()
    {
        // Stop the thread
        running_ = false;

        // Join the thread if it's joinable
        if (receiver_thread_.joinable())
        {
            receiver_thread_.join();
        }

        // Close UDP socket
        if (udp_client_)
        {
            udp_client_->close_socket();
        }

        RCLCPP_INFO(this->get_logger(), "OXTS driver shutdown complete");
    }

    void OxtsDriver::receiver_worker()
    {
        RCLCPP_INFO(this->get_logger(), "UDP receiver thread started");
        std::this_thread::sleep_for(std::chrono::seconds(1));

        while (running_)
        {
            if (!udp_client_->is_initialized())
            {
                // Try to reinitialize the socket if it's not open
                if (!udp_client_->initialize_socket())
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to initialize UDP socket, retrying in 1 second");
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
            }

            if (!broadcast_client_->is_initialized())
            {
                // Try to reinitialize the socket if it's not open
                if (!broadcast_client_->initialize_socket(true))
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to initialize Broadcast UDP socket, retrying in 1 second");
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
            }

            // Clear the buffer
            std::fill(data_buffer_.begin(), data_buffer_.end(), 0);

            // Receive data
            ssize_t bytes_read = udp_client_->read(data_buffer_.data(), BUFFER_SIZE);

            if (bytes_read > 0)
            {
                // Process the received data - thread-safe
                std::lock_guard<std::mutex> lock(data_mutex_);
                process_data(data_buffer_, static_cast<size_t>(bytes_read));
            }
            else if (bytes_read < 0)
            {
                // Error occurred
                RCLCPP_ERROR(this->get_logger(), "Error reading from UDP socket");
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            // For timeout (bytes_read == 0), just continue
        }

        RCLCPP_INFO(this->get_logger(), "UDP receiver thread stopped");
    }

    bool OxtsDriver::check_rate(double previous_timestamp, double current_timestamp)
    {
        bool skip_packet = false;
        // perform error checking on nrx timestamps
        if (current_timestamp <= DBL_EPSILON)
        {
            RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Time-stamp is \"zero\"; not initialized yet ...");
        }
        else if (current_timestamp - previous_timestamp > (2.01 / this->ncom_rate_))
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Packet drop detected. " << current_timestamp - previous_timestamp << " > " << (2.01 / this->ncom_rate_));
        }
        else if (current_timestamp < previous_timestamp)
        {
            if (std::abs(current_timestamp - previous_timestamp) > 10.0)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Packet timestamp desync, resetiing packet timestamps.\n\t%lf < %lf", current_timestamp, previous_timestamp);
            }
            else
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "Current packet is older than previous packet, skipping packet.\n\t%lf < %lf", current_timestamp, previous_timestamp);
                skip_packet = true;
            }
        }
        else if (current_timestamp == previous_timestamp)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Duplicate NCOM packet detected, skipping packet.");
            skip_packet = true;
        }
        else if (current_timestamp - previous_timestamp < (0.5 / this->ncom_rate_))
        {
            RCLCPP_WARN(this->get_logger(),
                        "Early packet detected, ncom_rate may be misconfigured.");
        }
        return skip_packet;
    }

    void OxtsDriver::process_data(const std::array<uint8_t, BUFFER_SIZE> &data, size_t bytes_read)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Data stream started");

        while (NComNewChars(nrx, &data[0], bytes_read) != COM_NEW_UPDATE)
        {
        }
        switch (nrx->mOutputPacketType)
        {
        case OUTPUT_PACKET_REGULAR:
            if (this->check_rate(this->last_week_second_, nrx->mTimeWeekSecond))
            {
                return;
            }

            this->last_week_second_ = nrx->mTimeWeekSecond;
            publish_ros_msgs();
            break;
        case OUTPUT_PACKET_STATUS:
            break;
        default:
            break;
        }
    }

    void OxtsDriver::publish_ros_msgs()
    {
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = imu_frame_id_;

        auto imu_msg = this->populate_imu(this->nrx, header);
        auto fix_msg = this->populate_fix(this->nrx, header);
        auto velocity_msg = this->populate_velocity(this->nrx, header);
        auto angular_acceleration = this->populate_angular_acceleration(this->nrx);
        auto time_reference_msg = this->populate_time_reference(this->nrx, header);
        if (this->gga_downsample_counter_ >= this->gga_counter_rollover_)
        {
            // If we do not have a fix, do not publish NMEA/GGA
            if (fix_msg.status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX)
            {
                std::string gga_str;
                this->generate_gga_string(fix_msg.latitude, fix_msg.longitude, fix_msg.altitude, header.stamp, gga_str);
                nmea_msgs::msg::Sentence gga_msg;
                gga_msg.sentence = gga_str;
                this->gga_pub_->publish(gga_msg);
                this->gga_downsample_counter_ = 0;
            }
        }
        else
        {
            this->gga_downsample_counter_++;
        }

        nav_msgs::msg::Odometry odom;
        odom.pose.covariance[0 * 6 + 0] = fix_msg.position_covariance[0];
        odom.pose.covariance[1 * 6 + 1] = fix_msg.position_covariance[4];
        odom.pose.covariance[2 * 6 + 2] = fix_msg.position_covariance[8];
        odom.pose.covariance[3 * 6 + 3] = imu_msg.orientation_covariance[0];
        odom.pose.covariance[4 * 6 + 4] = imu_msg.orientation_covariance[4];
        odom.pose.covariance[5 * 6 + 5] = imu_msg.orientation_covariance[8];
        this->log_imu2veh(this->nrx);

        bool success = this->process_kinematics(imu_msg, velocity_msg, angular_acceleration, fix_msg, odom);
        if (success)
        {
            // Don't publish data if processing failed (TF lookup for the lever arm failed)
            this->imu_pub_->publish(imu_msg);
            this->fix_pub_->publish(fix_msg);
            this->velocity_pub_->publish(velocity_msg);
            this->time_reference_pub_->publish(time_reference_msg);
            this->odom_pub_->publish(odom);
        }
    }

    sensor_msgs::msg::TimeReference OxtsDriver::populate_time_reference(const NComRxC *nrx, std_msgs::msg::Header header)
    {
        auto msg = sensor_msgs::msg::TimeReference();

        msg.header = std::move(header);

        msg.time_ref.sec = static_cast<int32_t>(nrx->mTimeWeekSecond);
        msg.time_ref.nanosec = static_cast<uint32_t>(
            (nrx->mTimeWeekSecond - std::floor(nrx->mTimeWeekSecond)) *
            NAV_CONST::SECS2NANOSECS);
        msg.source = "iTOW";

        return msg;
    }

    geometry_msgs::msg::TwistStamped OxtsDriver::populate_velocity(const NComRxC *nrx, std_msgs::msg::Header header)
    {
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header = std::move(header);

        // Resolve the NED velocity in the vehicle frame directly. The decoder's mIsoVo* fields are
        // actually FRD (not ISO FLU) despite their name, so they are not used here.
        auto enu_v = tf2::Vector3(nrx->mVe, nrx->mVn, -nrx->mVd);
        auto veh_o = getVehRPY(nrx) * getVat(); // ENU --> footprint
        auto veh_v = tf2::quatRotate(veh_o.inverse(), enu_v);

        auto sae_w = tf2::Vector3(nrx->mWx, nrx->mWy, nrx->mWz);
        auto veh_w = tf2::quatRotate(getVat(), sae_w);

        msg.twist.linear.x = veh_v.getX();
        msg.twist.linear.y = veh_v.getY();
        msg.twist.linear.z = veh_v.getZ();
        msg.twist.angular.x = NAV_CONST::DEG2RADS * veh_w.getX();
        msg.twist.angular.y = NAV_CONST::DEG2RADS * veh_w.getY();
        msg.twist.angular.z = NAV_CONST::DEG2RADS * veh_w.getZ();
        return msg;
    }

    geometry_msgs::msg::Vector3 OxtsDriver::populate_angular_acceleration(const NComRxC *nrx)
    {
        // The decoder derives angular acceleration by differentiating the angular rate, so use the
        // low-pass filtered version. Zero until the filter output is valid.
        auto msg = geometry_msgs::msg::Vector3();
        if (!(nrx->mIsFiltYxValid && nrx->mIsFiltYyValid && nrx->mIsFiltYzValid))
        {
            return msg;
        }

        auto sae_y = tf2::Vector3(nrx->mFiltYx, nrx->mFiltYy, nrx->mFiltYz); // Resolved in SAE frame
        auto veh_y = tf2::quatRotate(getVat(), sae_y);
        msg.x = NAV_CONST::DEG2RADS * veh_y.x();
        msg.y = NAV_CONST::DEG2RADS * veh_y.y();
        msg.z = NAV_CONST::DEG2RADS * veh_y.z();
        return msg;
    }

    void OxtsDriver::log_imu2veh(const NComRxC *nrx)
    {
        // Informational only: this rotation has already been applied by the INS to every output
        if (imu2veh_logged_ || !(nrx->mIsImu2VehHeadingValid && nrx->mIsImu2VehPitchValid && nrx->mIsImu2VehRollValid))
        {
            return;
        }
        imu2veh_logged_ = true;
        RCLCPP_INFO(this->get_logger(), "INS IMU-to-vehicle rotation (applied by NAVconfig): heading %.2f deg, pitch %.2f deg, roll %.2f deg",
                    nrx->mImu2VehHeading, nrx->mImu2VehPitch, nrx->mImu2VehRoll);
    }

    sensor_msgs::msg::NavSatFix OxtsDriver::populate_fix(const NComRxC *nrx, std_msgs::msg::Header header)
    {
        auto msg = sensor_msgs::msg::NavSatFix();
        msg.header = std::move(header);

        msg.status = populate_fix_status(nrx);

        msg.latitude = nrx->mLat;
        msg.longitude = nrx->mLon;
        msg.altitude = nrx->mAlt;

        // Square accuracy to get variance (could be incorrect)
        msg.position_covariance[0] = std::pow(nrx->mEastAcc, 2);
        msg.position_covariance[4] = std::pow(nrx->mNorthAcc, 2);
        msg.position_covariance[8] = std::pow(nrx->mAltAcc, 2);

        msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;

        return msg;
    }

    sensor_msgs::msg::NavSatStatus OxtsDriver::populate_fix_status(const NComRxC *nrx)
    {
        auto msg = sensor_msgs::msg::NavSatStatus();

        switch (nrx->mGpsPosMode)
        {
        // No Fix
        case NAV_CONST::GNSS_POS_MODE::NONE:
        case NAV_CONST::GNSS_POS_MODE::SEARCH:
        case NAV_CONST::GNSS_POS_MODE::DOPPLER:
        case NAV_CONST::GNSS_POS_MODE::NODATA:
        case NAV_CONST::GNSS_POS_MODE::BLANKED:
        case NAV_CONST::GNSS_POS_MODE::PP_DOPPLER:
        case NAV_CONST::GNSS_POS_MODE::NOT_KNOWN:
        case NAV_CONST::GNSS_POS_MODE::GX_DOPPLER:
        case NAV_CONST::GNSS_POS_MODE::IX_DOPPLER:
        case NAV_CONST::GNSS_POS_MODE::UNKNOWN:
            msg.status = msg.STATUS_NO_FIX;
            break;
        // Fix
        case NAV_CONST::GNSS_POS_MODE::SPS:
        case NAV_CONST::GNSS_POS_MODE::PP_SPS:
        case NAV_CONST::GNSS_POS_MODE::GX_SPS:
        case NAV_CONST::GNSS_POS_MODE::IX_SPS:
        case NAV_CONST::GNSS_POS_MODE::GENAID:
        case NAV_CONST::GNSS_POS_MODE::SEGMENT:
            msg.status = msg.STATUS_FIX;
            break;
        // Ground-based augmentation
        case NAV_CONST::GNSS_POS_MODE::DIFF:
        case NAV_CONST::GNSS_POS_MODE::FLOAT:
        case NAV_CONST::GNSS_POS_MODE::INTEGER:
        case NAV_CONST::GNSS_POS_MODE::PP_DIFF:
        case NAV_CONST::GNSS_POS_MODE::PP_FLOAT:
        case NAV_CONST::GNSS_POS_MODE::PP_INTEGER:
        case NAV_CONST::GNSS_POS_MODE::GX_DIFF:
        case NAV_CONST::GNSS_POS_MODE::GX_FLOAT:
        case NAV_CONST::GNSS_POS_MODE::GX_INTEGER:
        case NAV_CONST::GNSS_POS_MODE::IX_DIFF:
        case NAV_CONST::GNSS_POS_MODE::IX_FLOAT:
        case NAV_CONST::GNSS_POS_MODE::IX_INTEGER:
            msg.status = msg.STATUS_GBAS_FIX;
            break;
        // Satellite-based augmentation
        case NAV_CONST::GNSS_POS_MODE::WAAS:
        case NAV_CONST::GNSS_POS_MODE::OMNISTAR:
        case NAV_CONST::GNSS_POS_MODE::OMNISTARHP:
        case NAV_CONST::GNSS_POS_MODE::OMNISTARXP:
        case NAV_CONST::GNSS_POS_MODE::CDGPS:
        case NAV_CONST::GNSS_POS_MODE::PPP_CONVERGING:
        case NAV_CONST::GNSS_POS_MODE::PPP:
        case NAV_CONST::GNSS_POS_MODE::GX_SBAS:
        case NAV_CONST::GNSS_POS_MODE::IX_SBAS:
            msg.status = msg.STATUS_SBAS_FIX;
            break;
        default:
            msg.status = -2;
            break;
        }

        uint16_t service = 0;
        if (nrx->mIsGnssGpsEnabledValid && nrx->mGnssGpsEnabled)
            service |= msg.SERVICE_GPS;
        if (nrx->mIsGnssGlonassEnabledValid && nrx->mGnssGlonassEnabled)
            service |= msg.SERVICE_GLONASS;
        if (nrx->mIsGnssGalileoEnabledValid && nrx->mGnssGalileoEnabled)
            service |= msg.SERVICE_COMPASS;
        if (nrx->mIsGnssBeiDouEnabledValid && nrx->mGnssBeiDouEnabled)
            service |= msg.SERVICE_GALILEO;
        msg.service = service;

        return msg;
    }

    sensor_msgs::msg::Imu OxtsDriver::populate_imu(const NComRxC *nrx, std_msgs::msg::Header header)
    {
        auto msg = sensor_msgs::msg::Imu();
        msg.header = std::move(header);

        auto sae_o = tf2::Quaternion(); // Orientation of the SAE vehicle frame (ENU frame)
        auto sae_w = tf2::Vector3();    // Angular rate in SAE vehicle frame (rads)
        auto sae_a = tf2::Vector3();    // Linear Acceleration in the SAE vehicle frame (m/s^2)
        auto veh_o = tf2::Quaternion(); // Orientation of the vehicle (ENU frame)
        auto veh_w = tf2::Vector3();    // Angular rate in imu frame (rads)
        auto veh_a = tf2::Vector3();    // Linear Acceleration in the vehicle frame (m/s^2)

        // Get SAE --> robotics frame transformation --------------------------------
        auto sae_to_footprint = getVat();

        // Get imu orientation from NCOM packet -------------------------------------
        sae_o = getVehRPY(nrx);           // ENU --> SAE
        veh_o = sae_o * sae_to_footprint; // ENU --> footprint
        tf2::convert(veh_o, msg.orientation);

        // Covariance = 0 => unknown. -1 => invalid
        double roll_acc_rad = nrx->mRollAcc * NAV_CONST::DEG2RADS;
        double pitch_acc_rad = nrx->mPitchAcc * NAV_CONST::DEG2RADS;
        double heading_acc_rad = nrx->mHeadingAcc * NAV_CONST::DEG2RADS;
        msg.orientation_covariance[0] = roll_acc_rad * roll_acc_rad;
        msg.orientation_covariance[4] = pitch_acc_rad * pitch_acc_rad;
        msg.orientation_covariance[8] = heading_acc_rad * heading_acc_rad;

        // Rotate angular rate data before copying into message ----------------------
        sae_w = tf2::Vector3(nrx->mWx, nrx->mWy, nrx->mWz); // Resolved in SAE frame
        sae_w *= NAV_CONST::DEG2RADS;
        veh_w = tf2::quatRotate(sae_to_footprint, sae_w);
        msg.angular_velocity.x = veh_w.x();
        msg.angular_velocity.y = veh_w.y();
        msg.angular_velocity.z = veh_w.z();
        msg.angular_velocity_covariance[0] = 0.0; // Row major about x, y, z axes
        // ...
        msg.angular_velocity_covariance[8] = 0.0;

        // Rotate linear acceleration data -------------------------------------------
        sae_a = tf2::Vector3(nrx->mAx, nrx->mAy, nrx->mAz); // Resolved in SAE frame
        veh_a = tf2::quatRotate(sae_to_footprint, sae_a);
        msg.linear_acceleration.x = veh_a.x();
        msg.linear_acceleration.y = veh_a.y();
        msg.linear_acceleration.z = veh_a.z();

        msg.linear_acceleration_covariance[0] = 0.0; // Row major about x, y, z axes
        // ...
        msg.linear_acceleration_covariance[8] = 0.0;
        return msg;
    }

    void OxtsDriver::rtcm_callback(const rtcm_msgs::msg::Message::ConstSharedPtr msg)
    {
        if (this->udp_client_ != nullptr)
        {
            int nbytes = this->udp_client_->send(msg->message.data(), msg->message.size());
            if (nbytes < 0)
            {
                RCLCPP_ERROR_STREAM(get_logger(), "Failed to send RTCM message over UDP to " << this->udp_host_ << ":" << this->udp_port_);
            }
        }
        if (this->broadcast_client_ != nullptr)
        {
            int nbytes = this->broadcast_client_->send(msg->message.data(), msg->message.size());
            if (nbytes < 0)
            {
                RCLCPP_ERROR_STREAM(get_logger(), "Failed to send RTCM message over UDP to " << this->udp_host_ << ":" << this->udp_port_);
            }
            else
            {
                RCLCPP_DEBUG(get_logger(), "RTCM corrections up-to-date");
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(), "UDP Broadcast socket does not exist");
        }
    }

    tf2::Quaternion OxtsDriver::getVat()
    {
        // NCOM outputs are already resolved in the vehicle frame (FRD) configured in NAVconfig,
        // so only the fixed FRD --> FLU conversion is needed. Do not apply mImu2Veh* here;
        // the INS has already applied that rotation.
        tf2::Quaternion vat;
        vat.setRPY(M_PI, 0.0, 0.0);
        return vat;
    }

    tf2::Quaternion OxtsDriver::getVehRPY(const NComRxC *nrx)
    {
        auto rpyVehNED = tf2::Quaternion(); // Orientation of the vehicle (NED frame)
        auto rpyVehENU = tf2::Quaternion(); // Orientation of the vehicle (ENU frame)
        auto ned2enu = tf2::Quaternion();   // NED to ENU rotation

        rpyVehNED.setRPY(NAV_CONST::DEG2RADS * nrx->mRoll,
                         NAV_CONST::DEG2RADS * nrx->mPitch,
                         NAV_CONST::DEG2RADS * nrx->mHeading);
        // NED to ENU rotation
        ned2enu.setRPY(NAV_CONST::DEG2RADS * 180.0, 0, NAV_CONST::DEG2RADS * 90.0);
        // transform from NED to ENU
        rpyVehENU = ned2enu * rpyVehNED;
        return rpyVehENU;
    }

    void OxtsDriver::generate_gga_string(double latitude, double longitude, double altitude, const rclcpp::Time &utc_time, std::string &gga_msg)
    {
        std::stringstream msg;
        std::stringstream gga_payload;

        int lat_deg = (int)std::abs(latitude);
        float lat_min = 60.0 * fmod(std::abs(latitude), 1.0);
        int lon_deg = (int)std::abs(longitude);
        float lon_min = 60.0 * fmod(std::abs(longitude), 1.0);

        double utc_timestamp = utc_time.seconds();
        uint8_t utc_hours = (uint8_t)(fmod(utc_timestamp / 3600.0, 24.0));
        uint8_t utc_minutes = (uint8_t)(60.0 * fmod(utc_timestamp / 3600.0, 1.0));
        uint8_t utc_secs = (uint8_t)(3600.0 * fmod(utc_timestamp / 3600.0, 24.0) - 3600.0 * utc_hours - 60.0 * utc_minutes);

        gga_payload.precision(3);
        gga_payload << std::setfill('0');
        gga_payload << "GPGGA," << std::setw(2) << (int)utc_hours << std::setw(2) << (int)utc_minutes << std::setw(2) << (int)utc_secs << ",";
        gga_payload << std::setw(2) << lat_deg << std::fixed << std::setw(6) << lat_min;

        if (latitude > 0)
        {
            gga_payload << ",N,";
        }
        else
        {
            gga_payload << ",S,";
        }

        gga_payload << std::setw(3) << lon_deg << std::fixed << std::setw(6) << lon_min;
        if (longitude > 0)
        {
            gga_payload << ",E,";
        }
        else
        {
            gga_payload << ",W,";
        }

        gga_payload.precision(1);
        gga_payload << "1,4,1.5,";
        gga_payload << std::setfill('0');
        gga_payload << std::fixed << std::setw(4) << altitude;
        gga_payload << ",M,0,M,,";

        uint8_t checksum = 0;
        for (size_t i = 0; i < gga_payload.str().size(); i++)
        {
            checksum ^= gga_payload.str()[i];
        }

        msg << "$" << gga_payload.str() << "*" << std::hex << std::setfill('0') << std::setw(2) << std::uppercase << (int)checksum << "\r\n";
        gga_msg = msg.str();
    }

    double OxtsDriver::utm_grid_convergence(double lat_deg, double lon_deg, int zone_number)
    {
        constexpr double DEG2RAD = M_PI / 180.0;
        double central_meridian_deg = (zone_number - 1) * 6.0 - 180.0 + 3.0;
        double delta_lon = (lon_deg - central_meridian_deg) * DEG2RAD;
        return std::atan(std::tan(delta_lon) * std::sin(lat_deg * DEG2RAD));
    }

    bool OxtsDriver::init_kinematics_transformer(const std::string &ins_frame)
    {
        geometry_msgs::msg::Transform transform;
        try
        {
            transform = tf_buffer_->lookupTransform(this->base_frame_, ins_frame, rclcpp::Time()).transform;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
            return false;
        }

        // The INS already resolves its outputs in the vehicle frame (NAVconfig orientation), so the TF from
        // base to the INS frame must be a pure translation. Any rotation here would be silently ignored.
        Eigen::Quaterniond mount_rotation(transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
        double mount_rotation_deg = Eigen::AngleAxisd(mount_rotation.normalized()).angle() * 180.0 / M_PI;
        if (mount_rotation_deg > mount_rotation_tolerance_deg_)
        {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "TF from '%s' to '%s' has a %.3f deg rotation (tolerance %.3f deg). Configure the IMU "
                                  "orientation in NAVconfig so the INS outputs in the vehicle frame, and give '%s' an "
                                  "identity rotation relative to '%s'. Not publishing.",
                                  base_frame_.c_str(), ins_frame.c_str(), mount_rotation_deg, mount_rotation_tolerance_deg_,
                                  ins_frame.c_str(), base_frame_.c_str());
            return false;
        }

        Eigen::Vector3d lever_arm(transform.translation.x, transform.translation.y, transform.translation.z);
        kinematics_transformer_ = std::make_shared<KinematicsTransformer>(lever_arm);
        RCLCPP_INFO(this->get_logger(), "INS lever arm in '%s': [%.3f, %.3f, %.3f] m",
                    base_frame_.c_str(), lever_arm.x(), lever_arm.y(), lever_arm.z());
        return true;
    }

    bool OxtsDriver::process_kinematics(sensor_msgs::msg::Imu &imu,
                                      geometry_msgs::msg::TwistStamped &vel,
                                      const geometry_msgs::msg::Vector3 &ins_angular_acceleration,
                                      sensor_msgs::msg::NavSatFix &fix,
                                      nav_msgs::msg::Odometry &odom)
    {
        // INS measurement point position in UTM
        Eigen::Vector3d utm_ins_position;
        std::string zone;
        gps_tools::LLtoUTM(fix.latitude, fix.longitude, utm_ins_position.y(), utm_ins_position.x(), zone);
        utm_ins_position.z() = fix.altitude;
        int zone_number = std::stoi(zone);

        if (utm_zone_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "UTM zone: %s", zone.c_str());
        }
        else if (zone_number != std::stoi(utm_zone_))
        {
            RCLCPP_WARN(this->get_logger(), "UTM zone changed from %s to %s, odometry will jump", utm_zone_.c_str(), zone.c_str());
        }
        utm_zone_ = zone;

        // Orientation from the driver is relative to ENU (true north). Rotate into the UTM grid.
        Eigen::Quaterniond enu_orientation = {imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z};
        double convergence = utm_grid_convergence(fix.latitude, fix.longitude, zone_number);
        Eigen::Quaterniond utm_orientation = Eigen::AngleAxisd(convergence, Eigen::Vector3d::UnitZ()) * enu_orientation;

        Eigen::Vector3d ins_linear_velocity = {vel.twist.linear.x, vel.twist.linear.y, vel.twist.linear.z};
        Eigen::Vector3d angular_velocity = {imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z};
        Eigen::Vector3d ins_linear_acceleration = {imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z};
        Eigen::Vector3d angular_acceleration = {ins_angular_acceleration.x, ins_angular_acceleration.y, ins_angular_acceleration.z};

        const std::string frame_id = "utm";

        if (!kinematics_transformer_ && !init_kinematics_transformer(imu.header.frame_id))
        {
            return false;
        }

        // The base frame shares axes with the INS output frame, so orientation and angular velocity
        // pass through unchanged. Only the lever arm affects position, velocity and acceleration.
        Eigen::Vector3d utm_base_position = kinematics_transformer_->transform_position(utm_ins_position, utm_orientation);
        Eigen::Vector3d base_linear_velocity = kinematics_transformer_->transform_linear_velocity(ins_linear_velocity, angular_velocity);
        Eigen::Vector3d base_linear_acceleration = kinematics_transformer_->transform_linear_acceleration(ins_linear_acceleration, angular_velocity, angular_acceleration);

        /** IMU (orientation stays relative to ENU per REP-103) */
        imu.header.frame_id = base_frame_;
        imu.linear_acceleration = to_vector3_msg(base_linear_acceleration);

        vel.header.frame_id = base_frame_;
        vel.header.stamp = imu.header.stamp;
        vel.twist.linear = to_vector3_msg(base_linear_velocity);

        /** Odometry */
        odom.header.frame_id = frame_id;
        odom.child_frame_id = base_frame_;
        odom.header.stamp = imu.header.stamp;
        odom.pose.pose.position.x = utm_base_position.x();
        odom.pose.pose.position.y = utm_base_position.y();
        odom.pose.pose.position.z = utm_base_position.z();
        odom.pose.pose.orientation = to_quaternion_msg(utm_orientation);
        odom.twist.twist.linear = to_vector3_msg(base_linear_velocity);
        odom.twist.twist.angular = to_vector3_msg(angular_velocity);

        /** Fix */
        fix.header.frame_id = base_frame_;
        fix.header.stamp = imu.header.stamp;
        gps_tools::UTMtoLL(utm_base_position.y(), utm_base_position.x(), zone, fix.latitude, fix.longitude);
        fix.altitude = utm_base_position.z();

        /** TF */
        if (tf_broadcaster_ != nullptr && static_tf_broadcaster_ != nullptr)
        {
            geometry_msgs::msg::TransformStamped odom_tf;
            odom_tf.header.frame_id = frame_id;
            odom_tf.child_frame_id = base_frame_;
            odom_tf.header.stamp = imu.header.stamp;
            odom_tf.transform.translation.x = odom.pose.pose.position.x;
            odom_tf.transform.translation.y = odom.pose.pose.position.y;
            odom_tf.transform.translation.z = odom.pose.pose.position.z;
            odom_tf.transform.rotation = odom.pose.pose.orientation;
            tf_broadcaster_->sendTransform(odom_tf);

            // Local frame near the starting point so rviz does not lose precision on large UTM coordinates
            if (!rviz_frame_published_) {
                rviz_frame_published_ = true;

                geometry_msgs::msg::TransformStamped rviz_tf;
                rviz_tf.header.frame_id = frame_id;
                rviz_tf.header.stamp = imu.header.stamp;
                rviz_tf.child_frame_id = "rviz";
                rviz_tf.transform.translation.x = odom.pose.pose.position.x;
                rviz_tf.transform.translation.y = odom.pose.pose.position.y;
                rviz_tf.transform.translation.z = odom.pose.pose.position.z;
                rviz_tf.transform.rotation.w = 1.0;
                static_tf_broadcaster_->sendTransform(rviz_tf);
            }
        }

        return true;
    }

    UdpClient::UdpClient(const std::string &host, int port)
        : host_(host), port_(port)
    {
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("UdpClient"), "Failed to create UDP socket");
        }
    }

    bool UdpClient::initialize_socket(const bool broadcast)
    {
        if (socket_fd_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("UdpClient"), "Socket not created");
            return false;
        }

        // Set socket to non-blocking mode
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);

        // Setup server address
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(port_);

        if (inet_pton(AF_INET, host_.c_str(), &server_addr_.sin_addr) <= 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("UdpClient"), "[UDP] Invalid address: %s", host_.c_str());
            return false;
        }

        // Set up broadcast option
        int enableBroadcast = broadcast;
        int ret = setsockopt(socket_fd_, SOL_SOCKET, SO_BROADCAST, &enableBroadcast, sizeof(enableBroadcast));
        if (ret < 0)
            perror("UdpClient::initialize_socket (setsockopt(broadcast))");

        // For UDP, we need to bind the socket to receive data
        struct sockaddr_in client_addr;
        memset(&client_addr, 0, sizeof(client_addr));
        client_addr.sin_family = AF_INET;
        client_addr.sin_port = htons(port_);
        client_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(socket_fd_, (struct sockaddr *)&client_addr, sizeof(client_addr)) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("UdpClient"), "[UDP] Failed to bind to port %d", port_);
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("UdpClient"), "[UDP] Socket initialized on port %d", port_);
        initialized_ = true;
        return true;
    }

    void UdpClient::close_socket()
    {
        if (socket_fd_ >= 0)
        {
            close(socket_fd_);
            socket_fd_ = -1;
            initialized_ = false;
        }
    }

    bool UdpClient::is_initialized() const
    {
        return initialized_;
    }

    ssize_t UdpClient::read(void *buffer, size_t count)
    {
        struct timeval tv;
        tv.tv_sec = 1; // 1 second timeout
        tv.tv_usec = 0;

        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(socket_fd_, &readfds);

        int select_result = select(socket_fd_ + 1, &readfds, NULL, NULL, &tv);
        if (select_result <= 0)
        {
            if (select_result == 0)
            {
                // Timeout - no data available, but not necessarily an error in UDP
                return 0;
            }

            RCLCPP_ERROR(rclcpp::get_logger("UdpClient"), "Socket error occurred, resetting");
            this->close_socket();
            return -1;
        }

        socklen_t addrlen = sizeof(server_addr_);
        return recvfrom(socket_fd_, buffer, count, 0, (struct sockaddr *)&server_addr_, &addrlen);
    }

    ssize_t UdpClient::send(const void *buffer, size_t count)
    {
        ssize_t nbytes = sendto(socket_fd_, buffer, count, 0, (struct sockaddr *)&server_addr_, sizeof(server_addr_));
        if (nbytes < 0)
            perror("UdpClient::send");
        return nbytes;
    }

    UdpClient::~UdpClient()
    {
        if (socket_fd_ >= 0)
        {
            close(socket_fd_);
        }
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(oxts_ros2::OxtsDriver)
