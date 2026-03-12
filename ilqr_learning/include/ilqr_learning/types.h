/*
 * @Author: zhou hao polaris201809@163.com
 * @Date: 2026-03-04 10:20:16
 * @LastEditors: zhou hao polaris201809@163.com
 * @LastEditTime: 2026-03-12 13:25:33
 * @FilePath: /baymax_ilqr_test_ws/ilqr_learning/include/ilqr_learning/types.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <limits>
#include <vector>

constexpr double KMathEpsilon = 1e-6;

constexpr int kStateDim = 4;
constexpr int kControlDim = 2;
constexpr double ego_length = 15.0;
constexpr double ego_width = 3.2;

constexpr double alpha = 1.0;   // 罚函数因子

struct Vec2d {
  double x;
  double y;
  double theta;
  
  Vec2d() : x(0.0), y(0.0), theta(0.0) {}
};

struct Polygon {
  std::vector<Vec2d> points;
};

struct Obstacle {
  Vec2d position;  // 障碍物位置
  double radius;  // 避障半径
  Polygon polygon_points;   // 障碍物的多边形点
  double weight = 100.0;   // 障碍物权重
};
