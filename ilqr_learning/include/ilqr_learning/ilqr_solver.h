/*
 * @Author: zhou hao polaris201809@163.com
 * @Date: 2026-03-03 17:44:34
 * @LastEditors: zhou hao polaris201809@163.com
 * @LastEditTime: 2026-03-10 13:28:42
 * @FilePath: /baymax_ilqr_test_ws/ilqr_learning/include/ilqr_learning/ilqr_solver.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <vector>

#include "cost.h"
#include "dynamics.h"

class ILQRSolver {
 public:
  ILQRSolver(int horizon, const Dynamics& dynamics, const Cost& cost);

  void Solve(const Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>& u_traj);

  void SaveTrajectoryData(const std::vector<Eigen::VectorXd>& x_traj,
                          const std::vector<Eigen::VectorXd>& u_traj, double total_cost,
                          const std::string& filename) const;

 private:
  void Rollout(const Eigen::VectorXd& x0, const std::vector<Eigen::VectorXd>& u_traj,
               std::vector<Eigen::VectorXd>& x_traj, double& total_cost);

  void BackwardPass(const std::vector<Eigen::VectorXd>& x_traj,
                    const std::vector<Eigen::VectorXd>& u_traj, std::vector<Eigen::VectorXd>& k,
                    std::vector<Eigen::MatrixXd>& K, double regularization);

 private:
  int N_;
  Dynamics dynamics_;
  Cost cost_;
};