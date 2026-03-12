/*
 * @Author: zhou hao polaris201809@163.com
 * @Date: 2026-03-03 17:44:27
 * @LastEditors: zhou hao polaris201809@163.com
 * @LastEditTime: 2026-03-10 13:29:01
 * @FilePath: /baymax_ilqr_test_ws/ilqr_learning/include/ilqr_learning/dynamics.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <Eigen/Dense>

#include "types.h"

// constexpr int kStateDim = 4;
// constexpr int kControlDim = 2;

class Dynamics {
 public:
  Dynamics(double dt, double wheelbase);

  Eigen::VectorXd Step(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const;

  void Linearize(const Eigen::VectorXd& x, const Eigen::VectorXd& u, Eigen::MatrixXd& A,
                 Eigen::MatrixXd& B) const;

 private:
  double dt_;
  double L_;
};