/*
 * @Author: zhou hao polaris201809@163.com
 * @Date: 2026-03-03 17:44:38
 * @LastEditors: zhou hao polaris201809@163.com
 * @LastEditTime: 2026-03-11 18:07:28
 * @FilePath: /baymax_ilqr_test_ws/ilqr_learning/include/ilqr_learning/cost.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <Eigen/Dense>

#include "types.h"

// constexpr int kStateDim = 4;
// constexpr int kControlDim = 2;

class Cost {
 public:
  Cost();
  explicit Cost(const Eigen::VectorXd& x_target);
  Cost(const Eigen::VectorXd& x_target, const Eigen::VectorXd& q_diag,
       const Eigen::VectorXd& r_diag, const Eigen::VectorXd& qf_diag);

  Obstacle CreateLightObstacle(double x = 5.0, double y = 5.5, double radius = 0.7,
                               double weight = 0.6, int vertices_count = 5);

  void AddObstacle(const Obstacle& obstacle);

  double RunningCost(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const;

  double TerminalCost(const Eigen::VectorXd& x) const;

  double ObstacleCost(const Eigen::VectorXd& x) const;

  void QuadraticizeRunningCost(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                               Eigen::VectorXd& lx, Eigen::VectorXd& lu, Eigen::MatrixXd& lxx,
                               Eigen::MatrixXd& luu, Eigen::MatrixXd& lux) const;

  void QuadraticizeTerminalCost(const Eigen::VectorXd& x, Eigen::VectorXd& lx,
                                Eigen::MatrixXd& lxx) const;

  void QuadraticizeObstacleCost(const Eigen::VectorXd& x, Eigen::VectorXd& lx_ob,
                                Eigen::MatrixXd& lxx_ob) const;

  const Eigen::VectorXd& TargetState() const;
  const std::vector<Obstacle>& Obstacles() const;

 private:
  Eigen::MatrixXd Q_;
  Eigen::MatrixXd R_;
  Eigen::MatrixXd Qf_;
  Eigen::MatrixXd Qob_;
  Eigen::VectorXd x_target_;

  std::vector<Obstacle> obstacles_;
};
