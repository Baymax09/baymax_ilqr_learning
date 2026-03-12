/*
 * @Author: zhou hao polaris201809@163.com
 * @Date: 2026-03-03 17:44:15
 * @LastEditors: zhou hao polaris201809@163.com
 * @LastEditTime: 2026-03-12 13:50:24
 * @FilePath: /baymax_ilqr_test_ws/ilqr_learning/src/ilqr_node.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "ilqr_solver.h"
#include "rclcpp/rclcpp.hpp"

class ILQRNode : public rclcpp::Node {
 public:
  ILQRNode() : Node("ilqr_node") {
    RCLCPP_INFO(this->get_logger(), "iLQR node started");

    // 测试调用
    RunILQR();
  }

 private:
  void RunILQR() {
    const double dt = this->declare_parameter("dt", 0.05);
    const double wheelbase = this->declare_parameter("wheelbase", 2.5);
    const int horizon = this->declare_parameter("horizon", 200);
    const std::vector<double> x0_vec =
        this->declare_parameter("x0", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    const std::vector<double> target_vec =
        this->declare_parameter("target", std::vector<double>{10.0, 10.0, 0.0, 0.0});

    const std::vector<double> q_diag_vec =
        this->declare_parameter("q_diag", std::vector<double>{1.0, 1.0, 1.0, 0.5});
    const std::vector<double> r_diag_vec =
        this->declare_parameter("r_diag", std::vector<double>{0.15, 0.08});
    const std::vector<double> qf_diag_vec =
        this->declare_parameter("qf_diag", std::vector<double>{800.0, 800.0, 400.0, 200.0});

    const bool save_trajectory = this->declare_parameter("save_trajectory", false);
    const std::string trajectory_file =
        this->declare_parameter("trajectory_file", "ilqr_trajectory.txt");
    const double obstacle_x = this->declare_parameter("obstacle_x", 7.0);
    const double obstacle_y = this->declare_parameter("obstacle_y", 7.0);
    const double obstacle_radius = this->declare_parameter("obstacle_radius", 0.7);
    const double obstacle_weight = this->declare_parameter("obstacle_weight", 10.0);
    const int obstacle_vertices = this->declare_parameter("obstacle_vertices", 5);

    RCLCPP_INFO(this->get_logger(), "obstacle_position, x: %.6f, y: %.6f, radius: %.6f", obstacle_x,
                obstacle_y, obstacle_radius);

    // kStateDim：4     状态维度
    // kControlDim：2   控制维度
    if (x0_vec.size() != kStateDim || target_vec.size() != kStateDim ||
        q_diag_vec.size() != kStateDim || qf_diag_vec.size() != kStateDim ||
        r_diag_vec.size() != kControlDim) {
      RCLCPP_ERROR(this->get_logger(),
                   "Parameter size invalid. x0=%zu target=%zu q_diag=%zu qf_diag=%zu r_diag=%zu",
                   x0_vec.size(), target_vec.size(), q_diag_vec.size(), qf_diag_vec.size(),
                   r_diag_vec.size());
      return;
    }
    if (obstacle_radius <= 0.0) {
      RCLCPP_ERROR(this->get_logger(), "obstacle_radius must be positive, got %.6f",
                   obstacle_radius);
      return;
    }

    // 创建动力学模型对象
    Dynamics dynamics(dt, wheelbase);
    // 创建成本函数对象
    Eigen::VectorXd target(kStateDim);
    Eigen::VectorXd q_diag(kStateDim);
    Eigen::VectorXd r_diag(kControlDim);
    Eigen::VectorXd qf_diag(kStateDim);
    for (int i = 0; i < kStateDim; ++i) {
      target(i) = target_vec[i];
      q_diag(i) = q_diag_vec[i];
      qf_diag(i) = qf_diag_vec[i];
    }
    for (int i = 0; i < kControlDim; ++i) {
      r_diag(i) = r_diag_vec[i];
    }
    Cost cost(target, q_diag, r_diag, qf_diag);
    Obstacle obstacle = cost.CreateLightObstacle(obstacle_x, obstacle_y, obstacle_radius,
                                                 obstacle_weight, obstacle_vertices);
    cost.AddObstacle(obstacle);

    ILQRSolver solver(horizon, dynamics, cost);

    Eigen::VectorXd x0(kStateDim);
    for (int i = 0; i < kStateDim; ++i) {
      x0(i) = x0_vec[i];
    }

    std::vector<Eigen::VectorXd> u_traj(horizon);
    for (int i = 0; i < horizon; ++i) u_traj[i] = Eigen::VectorXd::Zero(kControlDim);

    solver.Solve(x0, u_traj);

    // 如果需要，保存轨迹数据
    if (save_trajectory) {
      // 重新计算最终轨迹
      std::vector<Eigen::VectorXd> x_traj(horizon + 1);
      double total_cost = 0.0;

      x_traj[0] = x0;
      for (int i = 0; i < horizon; ++i) {
        x_traj[i + 1] = dynamics.Step(x_traj[i], u_traj[i]);
        total_cost += cost.RunningCost(x_traj[i], u_traj[i]);
      }
      total_cost += cost.TerminalCost(x_traj[horizon]);

      solver.SaveTrajectoryData(x_traj, u_traj, total_cost, trajectory_file);
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ILQRNode>();
  (void)node;
  rclcpp::shutdown();
  return 0;
}
