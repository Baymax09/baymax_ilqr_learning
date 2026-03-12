/*
 * @Author: zhou hao polaris201809@163.com
 * @Date: 2026-03-03 17:44:01
 * @LastEditors: zhou hao polaris201809@163.com
 * @LastEditTime: 2026-03-12 15:06:41
 * @FilePath: /baymax_ilqr_test_ws/ilqr_learning/src/cost.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置:
 * https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "cost.h"

#include <algorithm>
#include <array>

namespace {
struct CircleCenter {
  double x;
  double y;
};

constexpr int kCircleCount = 3;

double CircleRadius() {
  return std::sqrt(std::pow(ego_length / 6.0, 2) + std::pow(ego_width / 2.0, 2));
}

std::array<CircleCenter, kCircleCount> BuildEgoCircles(const Eigen::VectorXd& x) {
  const double px = x(0);
  const double py = x(1);
  const double theta = x(2);
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  const std::array<double, kCircleCount> longitudinal_offsets = {-ego_length / 3.0, 0.0,
                                                                 ego_length / 3.0};

  std::array<CircleCenter, kCircleCount> circles{};
  for (int i = 0; i < kCircleCount; ++i) {
    circles[i].x = px + longitudinal_offsets[i] * c;
    circles[i].y = py + longitudinal_offsets[i] * s;
  }
  return circles;
}

double MinDistanceToObstacleEdge(const CircleCenter& circle, const Obstacle& obstacle) {
  if (obstacle.polygon_points.points.empty()) {
    const double dx = circle.x - obstacle.position.x;
    const double dy = circle.y - obstacle.position.y;
    return std::sqrt(dx * dx + dy * dy + KMathEpsilon);
  }

  double min_dist = std::numeric_limits<double>::infinity();
  for (const auto& point : obstacle.polygon_points.points) {
    const double dx = circle.x - point.x;
    const double dy = circle.y - point.y;
    const double dist = std::sqrt(dx * dx + dy * dy + KMathEpsilon);
    min_dist = std::min(min_dist, dist);
  }
  return min_dist;
}

double EgoObstacleDistance(const Eigen::VectorXd& x, const Obstacle& obstacle) {
  const auto circles = BuildEgoCircles(x);
  const double circle_radius = CircleRadius();

  double min_surface_dist = std::numeric_limits<double>::infinity();
  for (const auto& circle : circles) {
    const double edge_dist = MinDistanceToObstacleEdge(circle, obstacle);
    min_surface_dist = std::min(min_surface_dist, edge_dist - circle_radius);
  }
  return min_surface_dist;
}
}  // namespace

Cost::Cost() {
  // 状态量矩阵
  Q_ = Eigen::MatrixXd::Zero(kStateDim, kStateDim);
  // 控制量矩阵
  R_ = Eigen::MatrixXd::Zero(kControlDim, kControlDim);
  // 终端状态权重
  Qf_ = Eigen::MatrixXd::Zero(kStateDim, kStateDim);

  // 高效地初始化对角矩阵，即状态量、控制量和终端状态权重矩阵
  Q_.diagonal() << 1.0, 1.0, 1.0, 0.5;
  R_.diagonal() << 0.15, 0.08;
  Qf_.diagonal() << 800.0, 800.0, 400.0, 200.0;

  x_target_ = Eigen::VectorXd::Zero(kStateDim);
  x_target_ << 10.0, 10.0, 0.0, 0.0;
}

Cost::Cost(const Eigen::VectorXd& x_target) : Cost() {
  if (x_target.size() == kStateDim) {
    x_target_ = x_target;
  }
}

Cost::Cost(const Eigen::VectorXd& x_target, const Eigen::VectorXd& q_diag,
           const Eigen::VectorXd& r_diag, const Eigen::VectorXd& qf_diag)
    : Cost(x_target) {
  if (q_diag.size() == kStateDim) {
    Q_.diagonal() = q_diag;
  }
  if (r_diag.size() == kControlDim) {
    R_.diagonal() = r_diag;
  }
  if (qf_diag.size() == kStateDim) {
    Qf_.diagonal() = qf_diag;
  }
}

void Cost::AddObstacle(const Obstacle& obstacle) {
  obstacles_.emplace_back(obstacle);
}

Obstacle Cost::CreateLightObstacle(double x, double y, double radius, double weight,
                                   int vertices_count) {
  Obstacle obs;
  // 障碍物位置设在直线中点附近
  obs.position.x = x;
  obs.position.y = y;
  obs.position.theta = 0.0;
  obs.radius = radius;
  obs.weight = weight;

  // 生成正五边形顶点（围绕圆心，顺时针或逆时针均可）
  // 外接圆半径设为避障半径的0.8倍，使多边形不超出避障圆
  const double poly_radius = obs.radius * 0.8;
  vertices_count = std::max(3, vertices_count);

  // 生成5个顶点（逆时针顺序）
  for (int i = 0; i < vertices_count; ++i) {
    Vec2d point;
    const double angle = 2.0 * M_PI * i / vertices_count;  // 等分圆

    // 计算顶点坐标（相对于圆心）
    point.x = obs.position.x + poly_radius * std::cos(angle);
    point.y = obs.position.y + poly_radius * std::sin(angle);
    point.theta = 0.0;  // 多边形顶点通常不需要方向，设为0

    obs.polygon_points.points.push_back(point);
  }

  return obs;
}

/*
    cost包含：Q和R，写成了二次型
*/
double Cost::RunningCost(const Eigen::VectorXd& x, const Eigen::VectorXd& u) const {
  Eigen::VectorXd dx = x - x_target_;
  /*
    这里取出了状态量和控制量的总代价;
    dx.transpose() * Q_ * dx计算得到的结果是一个标量（1x1的矩阵），需要用(0,
    0)提取出来内部的double值 也可以采用.value()方法提取标量值
  */
  return (dx.transpose() * Q_ * dx)(0, 0) + (u.transpose() * R_ * u)(0, 0) + ObstacleCost(x);
}

/*
    cost包含：终端代价Qf，写成了二次型
*/
double Cost::TerminalCost(const Eigen::VectorXd& x) const {
  Eigen::VectorXd dx = x - x_target_;
  return (dx.transpose() * Qf_ * dx)(0, 0);
}

/*
    cost包含：障碍物的代价Qob_，写成了二次型
    为什么不写成矩阵的形式，类似于Running Cost函数里求cost的方式：
      1. 多个障碍物无法合并成一个矩阵
      2. 某些势函数形式无法写成二次型
*/
double Cost::ObstacleCost(const Eigen::VectorXd& x) const {
  double cost = 0.0;
  if (obstacles_.empty()) {
    return cost;
  }

  for (const auto& obstacle : obstacles_) {
    // 使用三圆模型与障碍物边界点的最小表面距离作为避障距离
    const double distance = EgoObstacleDistance(x, obstacle);
    if (distance < obstacle.radius) {
      // 软约束，采用指数函数
      cost += obstacle.weight * std::exp(-alpha * (distance - obstacle.radius));
    }
  }
  return cost;
}

// 同终端约束一样
void Cost::QuadraticizeRunningCost(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                                   Eigen::VectorXd& lx, Eigen::VectorXd& lu, Eigen::MatrixXd& lxx,
                                   Eigen::MatrixXd& luu, Eigen::MatrixXd& lux) const {
  // 计算状态偏差
  Eigen::VectorXd dx = x - x_target_;

  // 一阶梯度: lx = 2 * Q * dx, lu = 2 * R * u
  lx = 2.0 * Q_ * dx;
  lu = 2.0 * R_ * u;

  // 二阶海森矩阵: lxx = 2 * Q, luu = 2 * R
  lxx = 2.0 * Q_;
  luu = 2.0 * R_;

  // 交叉项梯度 (对于标准LQR问题，通常为零)
  lux = Eigen::MatrixXd::Zero(kControlDim, kStateDim);

  Eigen::VectorXd lx_obs = Eigen::VectorXd::Zero(kStateDim);
  Eigen::MatrixXd lxx_obs = Eigen::MatrixXd::Zero(kStateDim, kStateDim);
  QuadraticizeObstacleCost(x, lx_obs, lxx_obs);

  lx += lx_obs;
  lxx += lxx_obs;
}

void Cost::QuadraticizeTerminalCost(const Eigen::VectorXd& x, Eigen::VectorXd& lx,
                                    Eigen::MatrixXd& lxx) const {
  // 计算状态偏差
  Eigen::VectorXd dx = x - x_target_;

  // 一阶梯度: lx = 2 * Qf * dx
  /*
    TODO： @zhouhao 推导过程后续自行再进一步推导，这里暂时不过多展开
    推导的过程：
       https://www.volcengine.com/experience/ark?csid=excs-202603101457-%5BbQ-scX0-oGDoW7XrFNuoi%5D&mode=chat&modelId=doubao-seed-2-0-pro-260215&isHistory=true
  */
  lx = 2.0 * Qf_ * dx;

  // 二阶海森矩阵: lxx = 2 * Qf
  lxx = 2.0 * Qf_;
}

/*
  lx_ob: 障碍物的梯度矩阵
  lxx_ob: 障碍物的海森矩阵
*/
void Cost::QuadraticizeObstacleCost(const Eigen::VectorXd& x, Eigen::VectorXd& lx_ob,
                                    Eigen::MatrixXd& lxx_ob) const {
  lx_ob = Eigen::VectorXd::Zero(kStateDim);
  lxx_ob = Eigen::MatrixXd::Zero(kStateDim, kStateDim);
  if (obstacles_.empty()) {
    return;
  }

  // 三圆模型 + 最近边界点带来不可微切换，使用数值差分稳定计算梯度和海森
  const double eps = 1e-4;
  const double f0 = ObstacleCost(x);

  for (int i = 0; i < kStateDim; ++i) {
    Eigen::VectorXd x_plus = x;
    Eigen::VectorXd x_minus = x;
    x_plus(i) += eps;
    x_minus(i) -= eps;
    const double f_plus = ObstacleCost(x_plus);
    const double f_minus = ObstacleCost(x_minus);
    lx_ob(i) = (f_plus - f_minus) / (2.0 * eps);

    lxx_ob(i, i) = (f_plus - 2.0 * f0 + f_minus) / (eps * eps);
  }

  for (int i = 0; i < kStateDim; ++i) {
    for (int j = i + 1; j < kStateDim; ++j) {
      Eigen::VectorXd x_pp = x;
      Eigen::VectorXd x_pm = x;
      Eigen::VectorXd x_mp = x;
      Eigen::VectorXd x_mm = x;
      x_pp(i) += eps;
      x_pp(j) += eps;
      x_pm(i) += eps;
      x_pm(j) -= eps;
      x_mp(i) -= eps;
      x_mp(j) += eps;
      x_mm(i) -= eps;
      x_mm(j) -= eps;

      const double f_pp = ObstacleCost(x_pp);
      const double f_pm = ObstacleCost(x_pm);
      const double f_mp = ObstacleCost(x_mp);
      const double f_mm = ObstacleCost(x_mm);
      const double hij = (f_pp - f_pm - f_mp + f_mm) / (4.0 * eps * eps);
      lxx_ob(i, j) = hij;
      lxx_ob(j, i) = hij;
    }
  }
  return;
}

const Eigen::VectorXd& Cost::TargetState() const {
  return x_target_;
}

const std::vector<Obstacle>& Cost::Obstacles() const {
  return obstacles_;
}
