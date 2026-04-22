// Copyright (c) 2018 Intel Corporation
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#ifndef NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_
#define NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_

#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_vector.hpp"

/*
将所有 AMCL 相关类封装在 nav2_amcl命名空间中
避免与 ROS2 / 其他导航包符号冲突
*/ 
namespace nav2_amcl
{

/**
 * @class nav2_amcl::MotionModel
 * @brief An abstract motion model class
 */
/**
 * MotionModel是一个 抽象运动模型类，本身不实现具体运动模型
 * 只为不同机器人类型（差速、全向）提供统一接口
 * 作为 所有运动模型的基类
 */
class MotionModel
{
public:
  virtual ~MotionModel() = default;

  /**
   * 这是一个 纯虚函数
   * = 0表示：没有默认实现，派生类必须实现该函数
   * 函数作用：初始化运动模型的噪声参数
   * alpha1 旋转 → 旋转的噪声
   * alpha2 旋转 → 平移的噪声
   * alpha3 平移 → 平移的噪声
   * alpha4 平移 → 旋转的噪声
   * alpha5 旋转 → 平移的第二项噪声
   */
  /**
   * @brief An factory to create motion models
   * @param type Type of motion model to create in factory
   * @param alpha1 error parameters, see documentation
   * @param alpha2 error parameters, see documentation
   * @param alpha3 error parameters, see documentation
   * @param alpha4 error parameters, see documentation
   * @param alpha5 error parameters, see documentation
   * @return MotionModel A pointer to the motion model it created
   */
  virtual void initialize(
    double alpha1, double alpha2, double alpha3, double alpha4,
    double alpha5) = 0;

  /**
   * odometryUpdate() 纯虚函数
   * 功能概述：根据里程计数据更新粒子滤波器的粒子位姿（预测步）
   * pf_t * pf ：AMCL 的粒子滤波器结构体，包含：粒子数组、每个粒子的位姿 (x, y, yaw)、权重
   * const pf_vector_t & pose：机器人当前的里程计位姿
   * const pf_vector_t & delta：里程计的增量
   */
  /**
   * @brief Update on new odometry data
   * @param pf The particle filter to update
   * @param pose pose of robot in odometry update
   * @param delta change in pose in odometry update
   */
  virtual void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta) = 0;
};
}  // namespace nav2_amcl

#endif  // NAV2_AMCL__MOTION_MODEL__MOTION_MODEL_HPP_
