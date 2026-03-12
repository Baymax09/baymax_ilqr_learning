<!--
 * @Author: zhou hao polaris201809@163.com
 * @Date: 2026-03-09 17:42:10
 * @LastEditors: zhou hao polaris201809@163.com
 * @LastEditTime: 2026-03-09 18:01:00
 * @FilePath: /baymax_ilqr_test_ws/iLQR项目架构文档.md
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
-->
# iLQR学习项目架构文档

## 项目概述

这是一个基于ROS2的iLQR（迭代线性二次调节器）项目，用于车辆运动规划。项目采用C++实现，使用Eigen库进行矩阵运算。

## 项目整体架构

### 项目结构

```
baymax_ilqr_test_ws/
├── build/                     # 构建输出目录
├── ilqr_learning/            # 主要源代码包
│   ├── include/ilqr_learning/ # 头文件目录
│   │   ├── types.h           # 类型定义
│   │   ├── dynamics.h        # 动力学模型头文件
│   │   ├── cost.h            # 成本函数头文件
│   │   └── ilqr_solver.h     # iLQR求解器头文件
│   ├── src/                  # 源代码实现
│   │   ├── dynamics.cpp      # 动力学模型实现
│   │   ├── cost.cpp          # 成本函数实现
│   │   ├── ilqr_solver.cpp   # iLQR求解器实现
│   │   └── ilqr_node.cpp     # ROS2节点实现
│   ├── CMakeLists.txt        # CMake构建配置
│   └── package.xml           # ROS2包描述文件
├── install/                  # 安装目录
└── log/                      # 构建日志
```

## 各文件详细说明

### 1. 类型定义文件 (types.h)

```cpp
constexpr int kStateDim = 4;   // 状态维度：[x, y, theta, v]
constexpr int kControlDim = 2; // 控制维度：[加速度a, 转向角delta]
```

定义了系统的状态和控制维度：
- 状态向量：车辆位置(x,y)、航向角(theta)、速度(v)
- 控制向量：加速度(a)、转向角(delta)

### 2. 动力学模型 (dynamics.h/cpp)

实现了车辆的运动学模型，基于自行车模型：

**主要功能：**
- `Step()`: 计算给定状态和控制输入下的下一时刻状态
- `Linearize()`: 计算动力学模型的雅可比矩阵A和B，用于线性化

  - 具体的线性化计算：

  - | 公式                       | 名称         | 含义                                    |
    | -------------------------- | ------------ | --------------------------------------- |
    | `Qx = lx + Aᵀ * Vx`        | 状态梯度     | 当前成本梯度 + 传递来的下游梯度         |
    | `Qu = lu + Bᵀ * Vx`        | 控制梯度     | 当前成本对控制的梯度 + 传递来的下游梯度 |
    | `Qxx = lxx + Aᵀ * Vxx * A` | 状态海森矩阵 | 当前曲率 + 传递来的下游曲率             |
    | `Quu = luu + Bᵀ * Vxx * B` | 控制海森矩阵 | 当前控制曲率 + 传递来的下游曲率         |
    | `Qux = lux + Bᵀ * Vxx * A` | 交叉海森矩阵 | 状态-控制耦合项                         |


**模型方程：**
```
x_next = x + dt * v * cos(theta)
y_next = y + dt * v * sin(theta)
theta_next = theta + dt * v/L * tan(delta)
v_next = v + dt * a
```

其中：
- `dt`: 时间步长
- `L`: 车辆轴距
- `delta`: 转向角（限制在[-0.6, 0.6]弧度）

**实现细节：**
- 使用Clamp函数限制转向角范围，确保物理可行性
- 雅可比矩阵计算用于线性化，支持iLQR算法中的后向传递
- 状态向量维度为4，控制向量维度为2

### 3. 成本函数 (cost.h/cpp)

实现了二次型成本函数：

**主要功能：**
- `RunningCost()`: 计算运行时刻的成本
- `TerminalCost()`: 计算终端时刻的成本
- `QuadraticizeRunningCost()`: 计算运行时刻成本的一阶和二阶导数
- `QuadraticizeTerminalCost()`: 计算终端时刻成本的一阶和二阶导数

**成本函数形式：**
```
运行成本: (x-x_target)^T * Q * (x-x_target) + u^T * R * u
终端成本: (x-x_target)^T * Qf * (x-x_target)
```

其中权重矩阵：
```
Q = diag(1.0, 1.0, 1.0, 0.5)    # 状态权重
R = diag(0.15, 0.08)           # 控制权重
Qf = diag(800.0, 800.0, 400.0, 200.0)  # 终端状态权重
```

**设计考虑：**
- 终端权重Qf远大于运行权重Q，强调到达目标的重要性
- 控制权重R相对较小，允许必要的控制动作
- 速度权重较小，位置和航向角权重较大

### 4. iLQR求解器 (ilqr_solver.h/cpp)

实现了iLQR算法的核心逻辑：

**主要功能：**
- `Solve()`: 主求解函数，执行iLQR迭代优化
- `Rollout()`: 前向模拟，计算给定控制序列下的状态轨迹和总成本
- `BackwardPass()`: 后向传递，计算最优控制的反馈增益和前馈项

**算法流程：**
1. 初始化控制序列
2. 前向模拟计算状态轨迹
3. 后向传递计算控制修正量
4. 线搜索确定步长
5. 更新控制序列
6. 检查收敛条件

**关键特性：**
- 使用正则化确保数值稳定性
- 实现线搜索保证成本下降
- 控制输入限制：加速度[-2,2] m/s²，转向角[-0.6,0.6] rad
- 自适应正则化调整策略
- 多种收敛条件检查（成本下降、梯度范数）

**实现细节：**
- 前向传递使用ClampControl函数确保控制输入在物理限制内
- 后向传递使用LDLT分解求解线性系统，数值稳定
- 线搜索使用多个步长候选[1.0, 0.5, 0.25, 0.1, 0.05, 0.01]
- 正则化范围[1e-7, 1e6]，自适应调整

### 5. ROS2节点 (ilqr_node.cpp)

作为ROS2节点，提供与ROS系统的接口：

**主要功能：**
- 从ROS参数服务器读取配置参数
- 创建动力学模型、成本函数和iLQR求解器
- 执行iLQR求解并输出结果

**可配置参数：**
- `dt`: 时间步长（默认0.05秒）
- `wheelbase`: 车辆轴距（默认2.5米）
- `horizon`: 优化时域长度（默认200步）
- `x0`: 初始状态（默认[0,0,0,0]）
- `target`: 目标状态（默认[10,10,0,0]）

**使用方式：**
```bash
ros2 run ilqr_learning ilqr_node --ros-args -p dt:=0.1 -p horizon:=100
```

### 6. 构建配置文件

**CMakeLists.txt:**
- 定义了两个目标：`ilqr_core`（静态库）和`ilqr_node`（可执行文件）
- 链接了Eigen3库和ROS2的rclcpp
- 配置了编译选项和安装规则

**package.xml:**
- ROS2包描述文件
- 定义了包名、版本、维护者等信息
- 声明了依赖关系（ament_cmake, rclcpp等）

## 算法原理

iLQR（Iterative Linear Quadratic Regulator）是一种用于非线性系统轨迹优化的迭代算法：

1. **线性化**：在当前轨迹附近对非线性动力学模型进行线性化
2. **二次化**：将非线性成本函数近似为二次型
3. **求解LQR**：求解线性二次调节器问题，得到局部最优控制修正
4. **前向模拟**：使用修正后的控制重新模拟轨迹
5. **迭代更新**：重复上述过程直到收敛

### 算法优势

- 相比于直接优化，iLQR利用了系统动力学结构，计算效率高
- 通过序列二次规划方法处理非线性问题
- 生成反馈控制律，对扰动有鲁棒性
- 可以处理状态和控制约束

### 数学表述

给定非线性系统：
```
x_{k+1} = f(x_k, u_k)
```

和成本函数：
```
J = Σ_{k=0}^{N-1} l(x_k, u_k) + l_f(x_N)
```

iLQR通过迭代求解以下线性二次问题：
```
δx_{k+1} = A_k δx_k + B_k δu_k
δJ = Σ_{k=0}^{N-1} [δx_k^T Q_k δx_k + δu_k^T R_k δu_k + 2δx_k^T Q_k δu_k] + δx_N^T Q_f δx_N
```

## 应用场景

这个项目主要用于：
- 自动驾驶车辆的运动规划
- 轨迹跟踪控制
- 非线性系统优化控制

项目实现了一个完整的iLQR求解器框架，可以方便地扩展到不同的动力学模型和成本函数，适用于各种车辆控制场景。

## 扩展方向

1. **障碍物避障**：在成本函数中添加障碍物避障项
2. **动力学模型扩展**：考虑更复杂的车辆动力学
3. **多智能体**：扩展到多车协同规划
4. **模型预测控制**：结合MPC实现滚动优化
5. **学习组件**：结合机器学习方法学习动力学模型或成本函数

## 编译与运行

### 编译项目

```bash
cd ~/baymax_ilqr_test_ws
colcon build --packages-select ilqr_learning
source install/setup.bash
```

### 运行节点

```bash
ros2 run ilqr_learning ilqr_node
```

### 自定义参数运行

```bash
ros2 run ilqr_learning ilqr_node --ros-args -p dt:=0.1 -p horizon:=100 -p x0:="[0.0, 0.0, 0.0, 1.0]" -p target:="[5.0, 5.0, 0.0, 0.0]"
```

## 调试与分析

项目提供了详细的日志输出，包括：
- 每次迭代的成本变化
- 收敛状态
- 最终状态和误差
- 正则化参数调整

这些信息有助于分析算法性能和调试问题。

## 工程调用流程

### 整体调用流程图

```
main() (ilqr_node.cpp)
  ↓
rclcpp::init() → 创建ILQRNode节点
  ↓
ILQRNode构造函数 → RunILQR()
  ↓
读取ROS参数 → 创建Dynamics、Cost和ILQRSolver对象
  ↓
ILQRSolver::Solve() → 执行iLQR优化
  ↓
前向模拟(Rollout) → 后向传递(BackwardPass) → 线搜索 → 更新控制
  ↓
输出优化结果 → rclcpp::spin() → 节点运行
```

### 详细调用流程分析

#### 1. 程序启动流程

```cpp
// ilqr_node.cpp main函数
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);                    // 初始化ROS2
    auto node = std::make_shared<ILQRNode>();     // 创建ILQRNode节点
    rclcpp::spin(node);                           // 运行节点
    rclcpp::shutdown();                           // 关闭ROS2
    return 0;
}
```

#### 2. 节点初始化流程

```cpp
// ILQRNode构造函数
ILQRNode::ILQRNode() : Node("ilqr_node")
{
    RCLCPP_INFO(this->get_logger(), "iLQR node started");
    RunILQR();  // 调用iLQR求解函数
}
```

#### 3. 参数读取与对象创建

```cpp
// RunILQR()函数流程
void ILQRNode::RunILQR()
{
    // 1. 从ROS参数服务器读取配置
    const double dt = this->declare_parameter("dt", 0.05);
    const double wheelbase = this->declare_parameter("wheelbase", 2.5);
    const int horizon = this->declare_parameter("horizon", 200);
    const std::vector<double> x0_vec = this->declare_parameter("x0", std::vector<double>{0.0, 0.0, 0.0, 0.0});
    const std::vector<double> target_vec = this->declare_parameter("target", std::vector<double>{10.0, 10.0, 0.0, 0.0});
    
    // 2. 创建动力学模型对象
    Dynamics dynamics(dt, wheelbase);
    
    // 3. 创建成本函数对象
    Eigen::VectorXd target(kStateDim);
    for (int i = 0; i < kStateDim; ++i) {
        target(i) = target_vec[i];
    }
    Cost cost(target);
    
    // 4. 创建iLQR求解器对象
    ILQRSolver solver(horizon, dynamics, cost);
    
    // 5. 设置初始状态和控制序列
    Eigen::VectorXd x0(kStateDim);
    for (int i = 0; i < kStateDim; ++i) {
        x0(i) = x0_vec[i];
    }
    std::vector<Eigen::VectorXd> u_traj(horizon);
    for (int i = 0; i < horizon; ++i)
        u_traj[i] = Eigen::VectorXd::Zero(kControlDim);
    
    // 6. 调用求解器
    solver.Solve(x0, u_traj);
}
```

#### 4. iLQR求解器核心流程

```cpp
// ILQRSolver::Solve()函数流程
void ILQRSolver::Solve(const Eigen::VectorXd& x0, std::vector<Eigen::VectorXd>& u_traj)
{
    // 1. 初始化控制序列和状态轨迹
    std::vector<Eigen::VectorXd> x_traj(N_ + 1);
    std::vector<Eigen::VectorXd> x_candidate(N_ + 1);
    std::vector<Eigen::VectorXd> u_candidate(N_);
    std::vector<Eigen::VectorXd> k(N_);           // 前馈项
    std::vector<Eigen::MatrixXd> K(N_);           // 反馈增益
    
    // 2. 初始前向模拟，计算初始轨迹和成本
    Rollout(x0, u_traj, x_traj, total_cost);
    
    // 3. 迭代优化循环
    for (int iter = 0; iter < kMaxIter; ++iter) {
        // 3.1 后向传递，计算控制修正量
        BackwardPass(x_traj, u_traj, k, K, regularization);
        
        // 3.2 检查收敛条件（梯度范数）
        if (max_k_norm < kGradTol) {
            std::cout << "Converged by feedforward norm at iter " << iter << std::endl;
            break;
        }
        
        // 3.3 线搜索确定最佳步长
        for (double alpha : alphas) {
            // 3.3.1 前向模拟候选轨迹
            x_candidate[0] = x0;
            for (int i = 0; i < N_; ++i) {
                const Eigen::VectorXd dx = x_candidate[i] - x_traj[i];
                const Eigen::VectorXd u_new = u_traj[i] + alpha * k[i] + K[i] * dx;
                u_candidate[i] = ClampControl(u_new);
                x_candidate[i + 1] = dynamics_.Step(x_candidate[i], u_candidate[i]);
            }
            
            // 3.3.2 计算候选轨迹成本
            double candidate_cost = 0.0;
            for (int i = 0; i < N_; ++i) {
                candidate_cost += cost_.RunningCost(x_candidate[i], u_candidate[i]);
            }
            candidate_cost += cost_.TerminalCost(x_candidate[N_]);
            
            // 3.3.3 检查成本是否下降
            if (candidate_cost < best_cost) {
                best_cost = candidate_cost;
                accepted = true;
                break;
            }
        }
        
        // 3.4 更新轨迹和正则化参数
        if (accepted) {
            u_traj = u_candidate;
            x_traj = x_candidate;
            total_cost = best_cost;
            regularization = std::max(1e-7, regularization * 0.5);
            
            // 3.4.1 检查成本收敛
            if (cost_drop < kCostTol) {
                std::cout << "Converged by cost reduction at iter " << iter << std::endl;
                break;
            }
        } else {
            // 3.4.2 增加正则化参数
            regularization = std::min(1e6, regularization * 10.0);
        }
    }
    
    // 4. 输出最终结果
    Eigen::VectorXd x_final = x0;
    for (int i = 0; i < N_; ++i) {
        x_final = dynamics_.Step(x_final, u_traj[i]);
    }
    const Eigen::VectorXd final_error = x_final - cost_.TargetState();
    std::cout << "Final state: " << x_final.transpose() << std::endl;
    std::cout << "Final error: " << final_error.transpose() << std::endl;
}
```

#### 5. 前向模拟流程

```cpp
// Rollout函数流程
void ILQRSolver::Rollout(const Eigen::VectorXd& x0, 
                        const std::vector<Eigen::VectorXd>& u_traj,
                        std::vector<Eigen::VectorXd>& x_traj, 
                        double& total_cost)
{
    x_traj[0] = x0;                    // 设置初始状态
    total_cost = 0.0;
    
    for (int i = 0; i < N_; ++i) {
        // 1. 限制控制输入范围
        const Eigen::VectorXd u_sat = ClampControl(u_traj[i]);
        
        // 2. 计算下一时刻状态
        x_traj[i + 1] = dynamics_.Step(x_traj[i], u_sat);
        
        // 3. 累加运行成本
        total_cost += cost_.RunningCost(x_traj[i], u_sat);
    }
    
    // 4. 添加终端成本
    total_cost += cost_.TerminalCost(x_traj[N_]);
}
```

#### 6. 后向传递流程

```cpp
// BackwardPass函数流程
void ILQRSolver::BackwardPass(const std::vector<Eigen::VectorXd>& x_traj,
                             const std::vector<Eigen::VectorXd>& u_traj,
                             std::vector<Eigen::VectorXd>& k,
                             std::vector<Eigen::MatrixXd>& K,
                             double regularization)
{
    // 1. 初始化终端价值函数
    Eigen::VectorXd Vx, Vxx;
    cost_.QuadraticizeTerminalCost(x_traj[N_], Vx, Vxx);
    
    // 2. 从后向前遍历时间步
    for (int i = N_ - 1; i >= 0; --i) {
        // 2.1 计算当前时刻成本的梯度和海森矩阵
        Eigen::VectorXd lx, lu;
        Eigen::MatrixXd lxx, luu, lux;
        cost_.QuadraticizeRunningCost(x_traj[i], u_traj[i], lx, lu, lxx, luu, lux);
        
        // 2.2 线性化动力学模型
        Eigen::MatrixXd A, B;
        dynamics_.Linearize(x_traj[i], u_traj[i], A, B);
        
        // 2.3 计算Q函数的梯度和海森矩阵
        const Eigen::VectorXd Qx = lx + A.transpose() * Vx;
        const Eigen::VectorXd Qu = lu + B.transpose() * Vx;
        const Eigen::MatrixXd Qxx = lxx + A.transpose() * Vxx * A;
        const Eigen::MatrixXd Quu = luu + B.transpose() * Vxx * B;
        const Eigen::MatrixXd Qux = lux + B.transpose() * Vxx * A;
        
        // 2.4 添加正则化并求解线性系统
        Eigen::MatrixXd Quu_reg = Quu + regularization * Eigen::MatrixXd::Identity(kControlDim, kControlDim);
        Eigen::LDLT<Eigen::MatrixXd> ldlt(Quu_reg);
        
        // 2.5 计算前馈项和反馈增益
        k[i] = -ldlt.solve(Qu);
        K[i] = -ldlt.solve(Qux);
        
        // 2.6 更新价值函数
        Vx = Qx + K[i].transpose() * Quu * k[i] + K[i].transpose() * Qu + Qux.transpose() * k[i];
        Vxx = Qxx + K[i].transpose() * Quu * K[i] + K[i].transpose() * Qux + Qux.transpose() * K[i];
        Vxx = 0.5 * (Vxx + Vxx.transpose());  // 确保对称性
    }
}
```

### 关键函数调用关系

1. **Dynamics类调用关系**：
   - `ILQRSolver::Rollout()` → `Dynamics::Step()`
   - `ILQRSolver::BackwardPass()` → `Dynamics::Linearize()`

2. **Cost类调用关系**：
   - `ILQRSolver::Rollout()` → `Cost::RunningCost()` 和 `Cost::TerminalCost()`
   - `ILQRSolver::BackwardPass()` → `Cost::QuadraticizeRunningCost()` 和 `Cost::QuadraticizeTerminalCost()`

3. **ILQRSolver内部调用**：
   - `Solve()` → `Rollout()` 和 `BackwardPass()` 交替调用
   - `Rollout()` → `Dynamics::Step()` 和 `Cost::RunningCost()`
   - `BackwardPass()` → `Dynamics::Linearize()` 和 `Cost::QuadraticizeRunningCost()`

### 数据流分析

1. **初始化阶段**：
   - ROS参数 → Eigen向量 → 对象构造

2. **优化阶段**：
   - 控制序列 → 前向模拟 → 状态轨迹
   - 状态轨迹 → 后向传递 → 控制修正量
   - 控制修正量 → 线搜索 → 新控制序列

3. **输出阶段**：
   - 优化结果 → 终端输出 → 日志记录

这个调用流程清晰地展示了从程序启动到iLQR求解完成的整个过程，以及各个类和函数之间的调用关系和数据流动。

## 可视化工具

为了方便调试和展示iLQR算法的控制效果，项目提供了可视化工具。

### 可视化脚本功能

项目包含一个Python可视化脚本 `visualize_ilqr.py`，提供以下功能：

1. **轨迹可视化**：
   - 车辆在2D平面上的运动轨迹
   - 起点和终点标记
   - 目标点标记
   - 车辆方向箭头指示

2. **控制输入可视化**：
   - 加速度随时间变化曲线
   - 转向角随时间变化曲线
   - 控制输入限制线显示

3. **状态变量可视化**：
   - 位置(X,Y)随时间变化
   - 速度和航向角随时间变化

4. **成本收敛可视化**：
   - 迭代过程中总成本的变化
   - 线性和对数坐标显示

### 使用方法

#### 1. 生成轨迹数据

首先，运行iLQR节点并保存轨迹数据：

```bash
# 方法1: 使用提供的运行脚本
cd ~/baymax_ilqr_test_ws
chmod +x run_ilqr.sh
./run_ilqr.sh

# 方法2: 手动运行
ros2 run ilqr_learning ilqr_node \
    --ros-args \
    -p save_trajectory:=true \
    -p trajectory_file:="my_trajectory.txt" \
    -p dt:=0.05 \
    -p horizon:=200 \
    -p x0:="[0.0, 0.0, 0.0, 0.0]" \
    -p target:="[10.0, 10.0, 0.0, 0.0]"
```

#### 2. 可视化轨迹

```bash
# 基本可视化
python3 visualize_ilqr.py --trajectory ilqr_trajectory.txt

# 指定目标点
python3 visualize_ilqr.py --trajectory ilqr_trajectory.txt --target 10.0 10.0 0.0 0.0

# 保存图像
python3 visualize_ilqr.py --trajectory ilqr_trajectory.txt --save my_result

# 同时显示成本收敛曲线
python3 visualize_ilqr.py --trajectory ilqr_trajectory.txt --log ilqr_log.txt --show_cost

# 完整参数示例
python3 visualize_ilqr.py \
    --trajectory ilqr_results/ilqr_trajectory.txt \
    --log ilqr_results/ilqr_log.txt \
    --target 10.0 10.0 0.0 0.0 \
    --save ilqr_results/my_analysis \
    --show_cost
```

#### 3. 自定义参数运行

可以通过环境变量自定义运行脚本中的参数：

```bash
# 设置不同的初始状态和目标
X0="[0.0, 0.0, 0.0, 2.0]" TARGET="[5.0, 8.0, 1.57, 0.0]" ./run_ilqr.sh

# 设置不同的时间步长和时域长度
DT=0.1 HORIZON=100 ./run_ilqr.sh

# 设置不同的车辆参数
DT=0.05 WHEELBASE=3.0 ./run_ilqr.sh
```

### 可视化脚本参数说明

| 参数 | 简写 | 类型 | 说明 |
|------|------|------|------|
| `--trajectory` | `-t` | 字符串 | 轨迹文件路径 |
| `--log` | `-l` | 字符串 | 日志文件路径 |
| `--target` | 无 | 4个浮点数 | 目标状态 [x, y, theta, v] |
| `--save` | `-s` | 字符串 | 保存图像路径(不包含扩展名) |
| `--show_cost` | 无 | 标志 | 显示成本收敛曲线 |

### 轨迹文件格式

轨迹文件是文本文件，格式如下：

```
# 第一行: 时间步数N
200

# 接下来N行: 每行包含状态x,y,theta,v和控制a,delta
0.00 0.00 0.00 0.00 0.15 0.05
0.01 0.00 0.00 0.01 0.15 0.05
...

# 最后一行: 总成本
123.456
```

### 调试技巧

1. **分析轨迹质量**：
   - 检查终点是否接近目标点
   - 观察轨迹是否平滑
   - 检查控制输入是否在限制范围内

2. **成本收敛分析**：
   - 成本曲线应单调下降
   - 如果成本不下降或振荡，可能需要调整正则化参数
   - 观察收敛速度，评估算法效率

3. **参数调整**：
   - 如果轨迹不平滑，增加控制权重R
   - 如果无法到达目标，减小控制权重或增加终端权重Qf
   - 如果收敛慢，调整正则化初始值和调整策略

4. **问题诊断**：
   - 检查控制输入是否经常达到限制
   - 观察状态变量是否有异常跳变
   - 分析成本下降速度和最终误差

### 示例输出

运行可视化脚本后，将生成以下图表：

1. **轨迹图**：显示车辆在2D平面上的运动路径，包括起点、终点和方向
2. **控制输入图**：显示加速度和转向角随时间的变化
3. **状态变量图**：显示位置、速度和航向角随时间的变化
4. **成本收敛图**：显示优化过程中总成本的变化

这些可视化结果有助于直观理解iLQR算法的控制效果，并方便调试和参数调整。