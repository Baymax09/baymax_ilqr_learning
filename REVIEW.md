# Code Review Report

项目：`baymax_ilqr_learning`  
审查范围：`ilqr_learning/include`、`ilqr_learning/src`  
审查时间：2026-03-12

## 1. 发现的问题（按严重级别）

### [高] BackwardPass 与实际执行控制不一致（未统一饱和控制）
- 位置：
  - `ilqr_learning/src/ilqr_solver.cpp:212`（Rollout 中使用 `ClampControl`）
  - `ilqr_learning/src/ilqr_solver.cpp:254`（BackwardPass 中直接用 `u_traj[i]`）
- 问题：
  - 前向 rollout 和线搜索阶段使用了饱和后的控制，但后向二次化和线性化使用的是未饱和控制，导致导数模型与真实执行系统不一致。
- 影响：
  - iLQR 的局部二次模型失配，可能引起收敛慢、震荡，或线搜索频繁失败。
- 最小修复建议：
  - 在 `BackwardPass` 中对每步控制先做 `u_sat = ClampControl(u_traj[i])`，并将 `u_sat` 统一用于 `QuadraticizeRunningCost` 与 `dynamics_.Linearize`。

### [高] 多边形障碍距离计算仅取顶点距离，缺少“点到边”距离
- 位置：
  - `ilqr_learning/src/cost.cpp:52`
- 问题：
  - `MinDistanceToObstacleEdge` 对多边形障碍仅遍历顶点，未计算到线段边的最短距离。
- 影响：
  - 当最近点落在线段中点附近时会高估距离，进而低估障碍代价，存在碰撞风险。
- 最小修复建议：
  - 改为遍历多边形每条线段，计算“圆心到线段最短距离”，取全局最小值。

### [中] 障碍代价在阈值处不连续，影响二阶方法稳定性
- 位置：
  - `ilqr_learning/src/cost.cpp:182`
  - `ilqr_learning/src/cost.cpp:237`
- 问题：
  - 仅当 `distance < obstacle.radius` 才加罚，代价在边界处发生硬切换；随后又基于该函数做数值差分 Hessian。
- 影响：
  - 梯度/曲率在边界附近不稳定，可能导致线搜索拒绝率升高，或出现“抖动式”更新。
- 最小修复建议：
  - 使用全域平滑 barrier（如 softplus/平滑指数），至少在阈值附近设置连续可导过渡带。

### [中] 全局常量暴露在全局命名空间，扩展时有命名冲突风险
- 位置：
  - `ilqr_learning/include/ilqr_learning/types.h:14`
- 问题：
  - `kStateDim`、`kControlDim`、`alpha` 等常量位于全局命名空间。
- 影响：
  - 多包/多库链接时可读性和命名隔离性不足。
- 最小修复建议：
  - 放入项目命名空间（例如 `ilqr_learning`），并同步修改引用处。

### [低] 格式化检查未通过（uncrustify）
- 位置：
  - 多文件（`cost.h`、`dynamics.h`、`ilqr_solver.h`、`cost.cpp`、`dynamics.cpp`、`ilqr_node.cpp`、`ilqr_solver.cpp` 等）
- 问题：
  - `colcon test` 中 `uncrustify` 报告代码风格偏差。
- 影响：
  - CI 规范性失败，不影响算法语义但影响提交流水线质量。
- 最小修复建议：
  - 统一执行并提交格式化结果，保证 lint 全绿。

## 2. 本地验证记录

执行命令：

```bash
colcon test --packages-select ilqr_learning --event-handlers console_direct+
```

结果摘要：
- `cppcheck`: Passed
- `lint_cmake`: Passed
- `uncrustify`: Failed（多文件风格差异）
- `xmllint`: Failed（离线环境无法拉取 ROS schema：`http://download.ros.org/schema/package_format3.xsd`）

说明：
- `xmllint` 失败属于环境网络可达性问题，不是 `package.xml` 语义错误的充分证据；建议在可联网 CI 环境复测确认。

## 3. 建议优先级

1. 先修复“控制饱和一致性”与“障碍距离点到边计算”（高优先级，直接影响安全与收敛）。
2. 再处理“障碍代价平滑化”（提高稳定性与可优化性）。
3. 最后统一命名空间与格式化（工程质量与可维护性）。
