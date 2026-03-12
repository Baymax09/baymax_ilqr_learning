/*
 * @Author: zhou hao polaris201809@163.com
 * @Date: 2026-03-03 17:43:55
 * @LastEditors: zhou hao polaris201809@163.com
 * @LastEditTime: 2026-03-10 16:47:38
 * @FilePath: /baymax_ilqr_test_ws/ilqr_learning/src/dynamics.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "dynamics.h"

#include <algorithm>
#include <cmath>

namespace {
double Clamp(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(value, max_value));
}
}  // namespace

Dynamics::Dynamics(double dt, double wheelbase) : dt_(dt), L_(wheelbase) {}

Eigen::VectorXd Dynamics::Step(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
  // 下一个状态 x_next = [px_next, py_next, theta_next, v_next]
  Eigen::VectorXd x_next(kStateDim);

  double px = x(0);
  double py = x(1);
  double theta = x(2);
  double v = x(3);

  double a = u(0);
  // 控制限幅
  double delta = Clamp(u(1), -0.6, 0.6);

  /*
    运动学方程
    dt: 0.05s
  */
  x_next(0) = px + dt_ * v * std::cos(theta);
  x_next(1) = py + dt_ * v * std::sin(theta);
  x_next(2) = theta + dt_ * v / L_ * std::tan(delta);
  x_next(3) = v + dt_ * a;

  return x_next;
}

/*
  对非线性车辆动力学方程进行一阶泰勒展开
  本质上是在回答：“如果我在当前轨迹附近把控制量稍微改一点，后续状态会如何变化？”
*/
void Dynamics::Linearize(const Eigen::VectorXd& x, const Eigen::VectorXd& u, Eigen::MatrixXd& A,
                         Eigen::MatrixXd& B) const {
  /*
    车辆模型的状态变量：x = [px, py, theta, v]
    控制变量：u = [a, delta]
    车辆运动学方程：
          px_next = px + dt * v * cos(theta)
          py_next = py + dt * v * sin(theta)
          theta_next = theta + dt * v / L * tan(delta)
          v_next = v + dt * a
  */

  double theta = x(2);
  double v = x(3);

  // 前轮转角限幅
  double delta = Clamp(u(1), -0.6, 0.6);

  // 初始化状态量的雅可比矩阵A和控制量的雅可比矩阵B
  A = Eigen::MatrixXd::Zero(kStateDim, kStateDim);
  B = Eigen::MatrixXd::Zero(kStateDim, kControlDim);

  // 计算 A 矩阵 (df/dx)
  // f1 = px + dt * v * cos(theta)
  A(0, 0) = 1.0;                         // d(f1)/d(px)
  A(0, 2) = -dt_ * v * std::sin(theta);  // d(f1)/d(theta)
  A(0, 3) = dt_ * std::cos(theta);       // d(f1)/d(v)

  // f2 = py + dt * v * sin(theta)
  A(1, 1) = 1.0;                        // d(f2)/d(py)
  A(1, 2) = dt_ * v * std::cos(theta);  // d(f2)/d(theta)
  A(1, 3) = dt_ * std::sin(theta);      // d(f2)/d(v)

  // f3 = theta + dt * v / L * tan(delta)
  A(2, 2) = 1.0;                         // d(f3)/d(theta)
  A(2, 3) = dt_ / L_ * std::tan(delta);  // d(f3)/d(v)

  // f4 = v + dt * a
  A(3, 3) = 1.0;  // d(f4)/d(v)

  // 计算 B 矩阵 (df/du)
  // f1 = px + dt * v * cos(theta) - 对控制量无直接依赖
  B(0, 0) = 0.0;  // d(f1)/d(a)
  B(0, 1) = 0.0;  // d(f1)/d(delta)

  // f2 = py + dt * v * sin(theta) - 对控制量无直接依赖
  B(1, 0) = 0.0;  // d(f2)/d(a)
  B(1, 1) = 0.0;  // d(f2)/d(delta)

  // f3 = theta + dt * v / L * tan(delta)
  B(2, 0) = 0.0;                                                       // d(f3)/d(a)
  B(2, 1) = dt_ * v / L_ * (1.0 / std::cos(delta) / std::cos(delta));  // d(f3)/d(delta)

  // f4 = v + dt * a
  B(3, 0) = dt_;  // d(f4)/d(a)
  B(3, 1) = 0.0;  // d(f4)/d(delta)
}
