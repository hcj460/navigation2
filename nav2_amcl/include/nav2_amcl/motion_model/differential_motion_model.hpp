/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef NAV2_AMCL__MOTION_MODEL__DIFFERENTIAL_MOTION_MODEL_HPP_
#define NAV2_AMCL__MOTION_MODEL__DIFFERENTIAL_MOTION_MODEL_HPP_

#include "nav2_amcl/motion_model/motion_model.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_vector.hpp"

/*
将所有 AMCL 相关类封装在 nav2_amcl命名空间中
避免与 ROS2 / 其他导航包符号冲突
*/ 
namespace nav2_amcl
{
/* 
* 差速机器人运动模型
* 适用于 两轮差速驱动机器人（最常见的小车、移动底盘）
*MotionModel是一个抽象基类，定义了所有运动模型的统一接口
*/
class DifferentialMotionModel : public nav2_amcl::MotionModel
{
public:
/*
* 初始化运动模型参数
* alpha1  旋转噪声（旋转 → 旋转）
* alpha2  平移噪声（旋转 → 平移）
* alpha3  平移噪声（平移 → 平移）
* alpha4  旋转噪声（平移 → 旋转）
* alpha5  平移噪声（旋转 → 平移，第二项）
* 在 AMCL 中的作用，用于计算 粒子在里程计更新时的位姿采样方差
* 值越大 → 不确定性越大 → 粒子越分散
* 通常通过 amcl_diff.yaml配置
*/
  virtual void initialize(
    double alpha1, double alpha2, double alpha3, double alpha4,
    double alpha5);
/*
* 里程计更新，根据里程计变化量 delta，更新粒子滤波器 pf
* pf: 粒子滤波器 包含：粒子集合、权重、当前估计位姿
* pose: 当前位姿
* delta: 里程计增量
*/
  virtual void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta);

private:
/*
 * 保存运动模型噪声参数
 * 由 initialize()赋值
 * 在 odometryUpdate()中使用
 */
  double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
};
}  // namespace nav2_amcl
#endif  // NAV2_AMCL__MOTION_MODEL__DIFFERENTIAL_MOTION_MODEL_HPP_
