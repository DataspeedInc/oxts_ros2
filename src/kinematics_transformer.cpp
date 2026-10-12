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

#include "kinematics_transformer.hpp"

namespace oxts_ros2
{
    KinematicsTransformer::KinematicsTransformer(const Eigen::Vector3d &lever_arm)
        : lever_arm_(lever_arm)
    {
    }

    Eigen::Vector3d KinematicsTransformer::transform_position(const Eigen::Vector3d &ins_position, const Eigen::Quaterniond &world_orientation) const
    {
        return ins_position - world_orientation * lever_arm_;
    }

    Eigen::Vector3d KinematicsTransformer::transform_linear_velocity(const Eigen::Vector3d &ins_linear_velocity, const Eigen::Vector3d &angular_velocity) const
    {
        return ins_linear_velocity - angular_velocity.cross(lever_arm_);
    }

    Eigen::Vector3d KinematicsTransformer::transform_linear_acceleration(const Eigen::Vector3d &ins_linear_acceleration, const Eigen::Vector3d &angular_velocity,
                                                                         const Eigen::Vector3d &angular_acceleration) const
    {
        return ins_linear_acceleration
               - angular_acceleration.cross(lever_arm_)
               - angular_velocity.cross(angular_velocity.cross(lever_arm_));
    }
}
