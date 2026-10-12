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
#include <memory>
#include <eigen3/Eigen/Dense>

namespace oxts_ros2
{
    /**
     * @brief Moves INS measurements from the INS measurement point to the base frame origin.
     *
     * The INS (configured in NAVconfig) already resolves orientation, angular rate, acceleration and
     * velocity in the vehicle frame, and the driver converts that to the ROS FLU convention. The base
     * frame and the INS output frame therefore share the same axes, so only the lever arm between them
     * has to be compensated. No rotation is applied here.
     *
     * All body-frame vectors (velocities, accelerations, angular rates) are resolved in the base frame axes.
     */
    class KinematicsTransformer
    {
    public:
        using SharedPtr = std::shared_ptr<KinematicsTransformer>;

        /**
         * @param lever_arm Position of the INS measurement point in the base frame [m]
         */
        explicit KinematicsTransformer(const Eigen::Vector3d &lever_arm);
        ~KinematicsTransformer() = default;

        /**
         * @brief Base origin position in the world frame: p_b = p_ins - R_wb * r
         * @param ins_position Position of the INS measurement point in the world frame [m]
         * @param world_orientation Orientation of the base frame in the world frame
         */
        Eigen::Vector3d transform_position(const Eigen::Vector3d &ins_position, const Eigen::Quaterniond &world_orientation) const;

        /**
         * @brief Base origin linear velocity: v_b = v_ins - w x r
         * @param ins_linear_velocity Linear velocity of the INS measurement point [m/s]
         * @param angular_velocity Angular velocity of the rigid body [rad/s]
         */
        Eigen::Vector3d transform_linear_velocity(const Eigen::Vector3d &ins_linear_velocity, const Eigen::Vector3d &angular_velocity) const;

        /**
         * @brief Base origin linear acceleration: a_b = a_ins - alpha x r - w x (w x r)
         * @param ins_linear_acceleration Linear acceleration (specific force) of the INS measurement point [m/s^2]
         * @param angular_velocity Angular velocity of the rigid body [rad/s]
         * @param angular_acceleration Angular acceleration of the rigid body [rad/s^2]
         */
        Eigen::Vector3d transform_linear_acceleration(const Eigen::Vector3d &ins_linear_acceleration, const Eigen::Vector3d &angular_velocity,
                                                      const Eigen::Vector3d &angular_acceleration) const;

        const Eigen::Vector3d &get_lever_arm() const { return lever_arm_; }

    private:
        Eigen::Vector3d lever_arm_;
    };

}
