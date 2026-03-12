/*
 * @Author: zhou hao polaris201809@163.com
 * @Date: 2026-03-03 17:44:09
 * @LastEditors: zhou hao polaris201809@163.com
 * @LastEditTime: 2026-03-10 17:59:11
 * @FilePath: /baymax_ilqr_test_ws/ilqr_learning/src/ilqr_solver.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "ilqr_solver.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace {
// 限制数值在指定范围内
double Clamp(double value, double min_value, double max_value) {
  return std::max(min_value, std::min(value, max_value));
}

// 对控制输入进行饱和限制
Eigen::VectorXd ClampControl(const Eigen::VectorXd& u) {
  Eigen::VectorXd u_sat = u;
  u_sat(0) = Clamp(u_sat(0), -2.0, 2.0);  // 加速度限制 [m/s^2]
  u_sat(1) = Clamp(u_sat(1), -0.6, 0.6);  // 转向角限制 [rad]
  return u_sat;
}
}  // namespace

ILQRSolver::ILQRSolver(int horizon, const Dynamics& dynamics, const Cost& cost)
    : N_(horizon), dynamics_(dynamics), cost_(cost) {}

void ILQRSolver::Solve(const Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>& u_traj) {
  if (static_cast<int>(u_traj.size()) != N_) {
    u_traj.assign(N_, Eigen::VectorXd::Zero(kControlDim));
  }

  std::vector<Eigen::VectorXd> x_traj(N_ + 1);       // 状态轨迹
  std::vector<Eigen::VectorXd> x_candidate(N_ + 1);  // 候选状态轨迹
  std::vector<Eigen::VectorXd> u_candidate(N_);      // 候选控制轨迹
  std::vector<Eigen::VectorXd> k(N_);                // 前馈增益
  std::vector<Eigen::MatrixXd> K(N_);                // 反馈增益矩阵

  double total_cost = 0.0;
  // 前向模拟计算初始状态轨迹和总成本
  Rollout(x0, u_traj, x_traj, total_cost);

  constexpr int kMaxIter = 100; // 最大迭代次数限制
  constexpr int kMaxConsecutiveLineSearchFail = 8;
  constexpr double kCostTolAbs = 1e-4;
  constexpr double kCostTolRel = 1e-6;
  constexpr double kGradTol = 1e-4;
  double regularization = 1e-5; // 正则化参数，用于防止数值不稳定
  int consecutive_line_search_fail = 0;

  // 不断迭代，设置的最大迭代次数为kMaxIter = 100
  for (int iter = 0; iter < kMaxIter; ++iter) {
    // 每次迭代过程里，后向传递计算前馈增益和反馈增益矩阵
    BackwardPass(x_traj, u_traj, k, K, regularization);

    double max_k_norm = 0.0;
    for (int i = 0; i < N_; ++i) {
      // norm：计算矩阵的范数
      max_k_norm = std::max(max_k_norm, k[i].norm());
    }
    if (max_k_norm < kGradTol) {
      std::cout << "Converged by feedforward norm at iter " << iter << ", cost: " << total_cost
                << std::endl;
      break;
    }

    bool accepted = false;
    double best_cost = total_cost;
    // 线搜索多个步长来计算，确保每次迭代都能真正降低目标函数
    const double alphas[] = {1.0, 0.5, 0.25, 0.1, 0.05, 0.01};

    // important：线搜索核心部分 -> 找到一个合适的步长α，使得新轨迹的成本比旧轨迹低
    for (double alpha : alphas) {
      x_candidate[0] = x0;
      // 得到所有的轨迹以及根据前馈增益k和反馈增益K，来更新控制输入
      for (int i = 0; i < N_; ++i) {
        const Eigen::VectorXd dx = x_candidate[i] - x_traj[i];
        const Eigen::VectorXd u_new = u_traj[i] + alpha * k[i] + K[i] * dx;
        u_candidate[i] = ClampControl(u_new);
        // 根据运动学推导下一时空的状态
        x_candidate[i + 1] = dynamics_.Step(x_candidate[i], u_candidate[i]);
      }

      double candidate_cost = 0.0;
      for (int i = 0; i < N_; ++i) {
        candidate_cost += cost_.RunningCost(x_candidate[i], u_candidate[i]);
      }
      candidate_cost += cost_.TerminalCost(x_candidate[N_]);

      if (candidate_cost < best_cost) {
        // 线搜索核心，这里需要确保cost是下降的才会更新
        best_cost = candidate_cost;
        accepted = true;
        break;
      }
    }

    if (accepted) {
      consecutive_line_search_fail = 0;
      // 代价确实小了之后，更新新轨迹
      u_traj = u_candidate;
      x_traj = x_candidate;
      const double cost_drop = total_cost - best_cost;
      total_cost = best_cost;
      const double relative_drop = cost_drop / std::max(1.0, std::abs(total_cost));
      regularization = std::max(1e-7, regularization * 0.5);
      std::cout << "Iter: " << iter << " Cost: " << total_cost << " dCost: " << cost_drop
                << " reg: " << regularization << std::endl;
      if (cost_drop < kCostTolAbs || relative_drop < kCostTolRel) {
        std::cout << "Converged by cost reduction at iter " << iter
                  << ", rel_dCost: " << relative_drop << std::endl;
        // 成本收敛，退出迭代
        break;
      }
    } else {
      // 线搜索失败（所有步长都导致成本上升），增加正则化, 使矩阵Q_uu正定, 然后继续迭代
      ++consecutive_line_search_fail;
      regularization = std::min(1e6, regularization * 10.0);
      if (regularization >= 1e6) {
        std::cout << "Stop: regularization reached upper bound, cost: " << total_cost << std::endl;
        break;
      }
      if (consecutive_line_search_fail >= kMaxConsecutiveLineSearchFail) {
        std::cout << "Stop: line-search failed for " << consecutive_line_search_fail
                  << " consecutive iterations, cost: " << total_cost
                  << ", reg: " << regularization << std::endl;
        break;
      }
      std::cout << "Iter: " << iter
                << " warning: line-search rejected all candidate step sizes"
                << ", increase reg to " << regularization << std::endl;
    }
  }

  Eigen::VectorXd x_final = x0;
  for (int i = 0; i < N_; ++i) {
    // 拿更低代价的控制序列来更新状态x，得到最优轨迹
    x_final = dynamics_.Step(x_final, u_traj[i]);
  }
  const Eigen::VectorXd final_error = x_final - cost_.TargetState();
  std::cout << "Final state: " << x_final.transpose() << std::endl;
  std::cout << "Final error: " << final_error.transpose() << std::endl;
}

void ILQRSolver::SaveTrajectoryData(const std::vector<Eigen::VectorXd>& x_traj,
                                    const std::vector<Eigen::VectorXd>& u_traj, double total_cost,
                                    const std::string& filename) const {
  std::ofstream file(filename);
  if (!file.is_open()) {
    std::cerr << "无法打开文件: " << filename << std::endl;
    return;
  }

  // 设置输出精度
  file << std::fixed << std::setprecision(6);

  // 写入头部信息
  file << "# iLQR Trajectory Data" << std::endl;
  file << "# Horizon: " << N_ << std::endl;
  file << "# Format: x y theta v a delta" << std::endl;
  const auto& obstacles = cost_.Obstacles();
  file << "# ObstacleCount: " << obstacles.size() << std::endl;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const auto& obstacle = obstacles[i];
    file << "# Obstacle " << i << " center: " << obstacle.position.x << " " << obstacle.position.y
         << " theta: " << obstacle.position.theta << " radius: " << obstacle.radius
         << " weight: " << obstacle.weight << std::endl;
    file << "# Obstacle " << i << " polygon:";
    for (const auto& p : obstacle.polygon_points.points) {
      file << " " << p.x << "," << p.y;
    }
    file << std::endl;
  }

  // 写入每个时间步的状态和控制
  for (int i = 0; i < N_; ++i) {
    file << x_traj[i](0) << " " << x_traj[i](1) << " " << x_traj[i](2) << " " << x_traj[i](3) << " "
         << u_traj[i](0) << " " << u_traj[i](1) << std::endl;
  }

  // 写入最终状态（无控制）
  file << x_traj[N_](0) << " " << x_traj[N_](1) << " " << x_traj[N_](2) << " " << x_traj[N_](3)
       << " "
       << "0.0 0.0" << std::endl;

  // 写入总成本
  file << "# Total Cost: " << total_cost << std::endl;

  file.close();
  std::cout << "轨迹数据已保存到: " << filename << std::endl;
}

/*
    前向模拟计算初始状态轨迹和总成本
*/
void ILQRSolver::Rollout(const Eigen::VectorXd& x0, const std::vector<Eigen::VectorXd>& u_traj,
                         std::vector<Eigen::VectorXd>& x_traj, double& total_cost) {
  // 初始化状态轨迹
  x_traj[0] = x0;
  total_cost = 0.0;

  // 前向模拟计算状态轨迹和成本
  for (int i = 0; i < N_; ++i) {
    // 确保控制输入在物理限制内，输入包括加速度a和（前轮）转角delta
    const Eigen::VectorXd u_sat = ClampControl(u_traj[i]);
    // 得到在时域N_里的所有前向轨迹：根据运动学方程推导而来
    x_traj[i + 1] = dynamics_.Step(x_traj[i], u_sat);  // 单步rollout
    // 计算得到Q和R的cost和
    total_cost += cost_.RunningCost(x_traj[i], u_sat);
  }
  // 添加终端成本
  total_cost += cost_.TerminalCost(x_traj[N_]);
}

/// @brief 后向传递计算前馈增益和反馈增益矩阵
/// @param x_traj 状态轨迹
/// @param u_traj 控制轨迹
/// @param k 前馈增益向量：表示在参考轨迹上的基础控制调整量
/// @param K 反馈增益矩阵：表示根据状态偏差调整控制的反馈系数
/// @param regularization 正则化参数，用于防止数值不稳定：1e-5
void ILQRSolver::BackwardPass(const std::vector<Eigen::VectorXd>& x_traj,
                              const std::vector<Eigen::VectorXd>& u_traj,
                              std::vector<Eigen::VectorXd>& k, std::vector<Eigen::MatrixXd>& K,
                              double regularization) {
  // 计算终端成本函数的梯度和海森矩阵
  Eigen::VectorXd lx_terminal;  // 一阶
  Eigen::MatrixXd lxx_terminal; // 二阶
  // 终端hession矩阵
  cost_.QuadraticizeTerminalCost(x_traj[N_], lx_terminal, lxx_terminal);

  // 初始化价值函数的梯度和海森矩阵
  Eigen::VectorXd Vx = lx_terminal;
  Eigen::MatrixXd Vxx = lxx_terminal;

  // 从后向前遍历时间步
  for (int i = N_ - 1; i >= 0; --i) {
    // 计算当前步的运行成本函数的梯度和海森矩阵
    Eigen::VectorXd lx; // 当前时刻即时成本对状态的梯度
    Eigen::VectorXd lu; // 当前时刻即时成本对控制的梯度
    Eigen::MatrixXd lxx; // 当前时刻即时成本对状态的海森矩阵
    Eigen::MatrixXd luu; // 当前时刻即时成本对控制的海森矩阵
    Eigen::MatrixXd lux; // 当前时刻即时成本对状态和控制的混合项
    /*
      因为前向rollout了N_步，所以这里是有终端的轨迹了
      获取从终点往回，各个点的运行成本函数的梯度（一阶导）和海森矩阵（二阶导）
    */
    cost_.QuadraticizeRunningCost(x_traj[i], u_traj[i], lx, lu, lxx, luu, lux);

    // 线性化动力学模型
    Eigen::MatrixXd A(kStateDim, kStateDim);
    Eigen::MatrixXd B(kStateDim, kControlDim);
    // 在当前状态和当前控制输入的一阶线性化
    dynamics_.Linearize(x_traj[i], u_traj[i], A, B);

    // 计算Q函数的梯度和海森矩阵，这里需要考虑线搜索，所以才有这个A和B
    // TODO： @zhouhao 需要推导一下这个方程为什么是这样的
    const Eigen::VectorXd Qx = lx + A.transpose() * Vx; // A的转置*Vx：下一时刻价值函数对当前状态的"传递"梯度
    const Eigen::VectorXd Qu = lu + B.transpose() * Vx; // B的转置*Vx：下一时刻价值函数对当前控制的"传递"梯度
    const Eigen::MatrixXd Qxx = lxx + A.transpose() * Vxx * A;  // 将下一时刻的状态二阶曲率信息传递到当前时刻
    const Eigen::MatrixXd Quu = luu + B.transpose() * Vxx * B;
    const Eigen::MatrixXd Qux = lux + B.transpose() * Vxx * A;

    // 添加正则化项确保正定性
    Eigen::MatrixXd Quu_reg = Quu;
    // Identity：生成动态大小双精度单位矩阵
    Quu_reg += regularization * Eigen::MatrixXd::Identity(kControlDim, kControlDim);

    /*
      使用LDLT分解求解线性系统，是求解对称正定矩阵线性方程组的高效方法
      - L：下三角矩阵（单位下三角）
      - D：对角矩阵
      - Lᵀ：L的转置（上三角）
    */
    Eigen::LDLT<Eigen::MatrixXd> ldlt(Quu_reg);
    if (ldlt.info() != Eigen::Success) {
      // 如果分解失败，增加正则化参数
      /*
        可能的失败原因：
          矩阵不是正定的（特征值为负或接近零）
          矩阵是奇异的（行列式为零）
          数值精度问题
      */
      Quu_reg += 1e-3 * Eigen::MatrixXd::Identity(kControlDim, kControlDim);
      ldlt.compute(Quu_reg);
    }

    // 计算前馈和反馈增益
    k[i] = -ldlt.solve(Qu);
    K[i] = -ldlt.solve(Qux);

    // 更新价值函数的梯度和海森矩阵
    Vx = Qx + K[i].transpose() * Quu * k[i] + K[i].transpose() * Qu + Qux.transpose() * k[i];
    Vxx = Qxx + K[i].transpose() * Quu * K[i] + K[i].transpose() * Qux + Qux.transpose() * K[i];

    // 确保海森矩阵对称
    Vxx = 0.5 * (Vxx + Vxx.transpose());
  }
}
