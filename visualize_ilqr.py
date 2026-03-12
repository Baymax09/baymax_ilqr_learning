#!/usr/bin/env python3
"""
iLQR可视化脚本
用于可视化iLQR算法的控制效果，包括轨迹、控制输入和成本变化
"""

import argparse
import os
import warnings

warnings.filterwarnings("ignore", message="Unable to import Axes3D.*")

import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle
from matplotlib.lines import Line2D

def configure_plot_fonts():
    chinese_fonts = [
        "Noto Sans CJK SC",
        "WenQuanYi Micro Hei",
        "SimHei",
        "Microsoft YaHei",
    ]
    for font_name in chinese_fonts:
        try:
            fm.findfont(font_name, fallback_to_default=False)
            plt.rcParams["font.sans-serif"] = [font_name, "DejaVu Sans"]
            plt.rcParams["axes.unicode_minus"] = False
            return True
        except ValueError:
            continue

    plt.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    return False


USE_CHINESE = configure_plot_fonts()


def tr(cn_text, en_text):
    return cn_text if USE_CHINESE else en_text


def compute_box_corners(cx, cy, theta, length, width):
    half_l = 0.5 * length
    half_w = 0.5 * width
    corners_local = np.array(
        [
            [half_l, half_w],
            [half_l, -half_w],
            [-half_l, -half_w],
            [-half_l, half_w],
            [half_l, half_w],
        ],
        dtype=float,
    )
    c = np.cos(theta)
    s = np.sin(theta)
    rot = np.array([[c, -s], [s, c]], dtype=float)
    corners_world = corners_local @ rot.T
    corners_world[:, 0] += cx
    corners_world[:, 1] += cy
    return corners_world


def parse_target_args(target_args):
    """
    兼容两种目标状态输入:
    1) --target 10 10 0 0
    2) --target 10,10,0,0
    """
    if target_args is None:
        return [10.0, 10.0, 0.0, 0.0]

    if len(target_args) == 1 and "," in target_args[0]:
        parts = [p for p in target_args[0].split(",") if p]
    else:
        parts = target_args

    if len(parts) != 4:
        raise ValueError(tr("参数 --target 需要 4 个值: x y theta v",
                            "argument --target requires 4 values: x y theta v"))

    return [float(v) for v in parts]

def parse_trajectory_file(filename):
    """
    解析轨迹文件，提取状态和控制序列
    
    文件格式:
    - 第一行: 时间步数N
    - 接下来N行: 每行包含状态x,y,theta,v和控制a,delta
    - 最后一行: 总成本
    """
    if not os.path.exists(filename):
        print(f"错误: 文件 {filename} 不存在")
        return None, None, None, []
    
    try:
        with open(filename, 'r') as f:
            lines = [line.strip() for line in f.readlines()]

        data = []
        total_cost = None
        declared_horizon = None
        obstacles = []
        obstacles_by_index = {}

        for line in lines:
            if not line:
                continue
            if line.startswith("#"):
                if "Horizon:" in line:
                    try:
                        declared_horizon = int(line.split("Horizon:")[1].strip())
                    except (ValueError, IndexError):
                        pass
                if "Total Cost:" in line:
                    try:
                        total_cost = float(line.split("Total Cost:")[1].strip())
                    except (ValueError, IndexError):
                        pass
                if line.startswith("# Obstacle ") and " center:" in line:
                    # 格式:
                    # 新格式: # Obstacle i center: x y theta: t radius: r weight: w
                    # 兼容旧格式: # Obstacle i center: x y radius: r weight: w
                    try:
                        header, payload = line.split("center:", 1)
                        idx = int(header.split()[2])
                        theta = 0.0
                        if "theta:" in payload:
                            center_part, remain = payload.split("theta:", 1)
                            theta_part, remain = remain.split("radius:", 1)
                            theta = float(theta_part.strip())
                        else:
                            center_part, remain = payload.split("radius:", 1)
                        radius_part, weight_part = remain.split("weight:", 1)
                        center_vals = center_part.strip().split()
                        obstacle = {
                            "index": idx,
                            "center": [float(center_vals[0]), float(center_vals[1])],
                            "theta": theta,
                            "radius": float(radius_part.strip()),
                            "weight": float(weight_part.strip()),
                            "polygon": [],
                        }
                        obstacles_by_index[idx] = obstacle
                    except (ValueError, IndexError):
                        pass
                if line.startswith("# Obstacle ") and " polygon:" in line:
                    # 格式: # Obstacle i polygon: x1,y1 x2,y2 ...
                    try:
                        header, payload = line.split("polygon:", 1)
                        idx = int(header.split()[2])
                        obstacle = obstacles_by_index.setdefault(
                            idx,
                            {"index": idx, "center": None, "theta": 0.0, "radius": None, "weight": None, "polygon": []},
                        )
                        points = []
                        for token in payload.strip().split():
                            xy = token.split(",")
                            if len(xy) != 2:
                                continue
                            points.append([float(xy[0]), float(xy[1])])
                        obstacle["polygon"] = points
                    except (ValueError, IndexError):
                        pass
                continue

            parts = line.split()
            # 兼容旧格式: 第一行为N
            if len(parts) == 1 and declared_horizon is None and not data:
                try:
                    declared_horizon = int(parts[0])
                    continue
                except ValueError:
                    pass

            if len(parts) >= 6:
                try:
                    data.append([float(x) for x in parts[:6]])
                except ValueError:
                    continue

        if not data:
            print(tr("错误: 轨迹文件中未找到有效轨迹数据",
                     "Error: no valid trajectory data found in file"))
            return None, None, None, []

        if declared_horizon is not None and len(data) not in (declared_horizon, declared_horizon + 1):
            print(tr(f"警告: 数据点数({len(data)})与Horizon({declared_horizon})不匹配",
                     f"Warning: data points ({len(data)}) mismatch horizon ({declared_horizon})"))

        data = np.array(data)
        states = data[:, :4]  # x, y, theta, v
        controls = data[:, 4:6]  # a, delta

        if total_cost is None:
            total_cost = float("nan")

        if obstacles_by_index:
            obstacles = [obstacles_by_index[k] for k in sorted(obstacles_by_index.keys())]
        return states, controls, total_cost, obstacles

    except Exception as e:
        print(f"{tr('解析文件时出错', 'Error parsing file')}: {e}")
        return None, None, None, []

def parse_log_file(filename):
    """
    解析日志文件，提取成本变化信息
    """
    if not os.path.exists(filename):
        print(f"警告: 日志文件 {filename} 不存在")
        return [], []
    
    iterations = []
    costs = []
    
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
        
        for line in lines:
            if "Iter:" in line and "Cost:" in line:
                parts = line.split()
                iter_idx = parts.index("Iter:") + 1
                cost_idx = parts.index("Cost:") + 1
                try:
                    iter_num = int(parts[iter_idx].strip(','))
                    cost_val = float(parts[cost_idx].strip(','))
                    iterations.append(iter_num)
                    costs.append(cost_val)
                except (ValueError, IndexError):
                    continue
        
        return iterations, costs
    
    except Exception as e:
        print(f"解析日志文件时出错: {e}")
        return [], []

def visualize_trajectory(states, controls, target=None, obstacles=None, ego_length=15.0,
                         ego_width=3.2, save_path=None):
    """
    可视化轨迹和控制输入
    """
    if states is None or controls is None:
        print(tr("错误: 状态或控制数据为空", "Error: states or controls are empty"))
        return
    
    # 创建图形
    plt.figure(figsize=(15, 12))
    
    # 1. 轨迹图 (x, y)
    ax_xy = plt.subplot(2, 2, 1)
    ax_xy.plot(states[:, 0], states[:, 1], 'b-', linewidth=2,
               label=tr('蓝线：车辆轨迹', 'Blue: trajectory'))
    ax_xy.plot(states[0, 0], states[0, 1], 'go', markersize=10,
               label=tr('绿点：起点', 'Green dot: start'))
    ax_xy.plot(states[-1, 0], states[-1, 1], 'ro', markersize=10,
               label=tr('红点：终点', 'Red dot: end'))
    
    if target is not None:
        ax_xy.plot(target[0], target[1], 'r*', markersize=15,
                   label=tr('红星：目标点', 'Red star: target'))

    if obstacles:
        for i, obstacle in enumerate(obstacles):
            center = obstacle.get("center")
            theta = obstacle.get("theta", 0.0)
            radius = obstacle.get("radius")
            polygon = obstacle.get("polygon", [])

            if center is not None:
                ax_xy.plot(
                    center[0], center[1], marker='x', color='k', markersize=9, markeredgewidth=2,
                    label=tr('黑叉：障碍物中心', 'Black x: obstacle center') if i == 0 else None)
                if radius is not None and radius > 0.0:
                    ax_xy.add_patch(
                        Circle((center[0], center[1]), radius, fill=False, edgecolor='k',
                               linestyle='--', linewidth=1.2, alpha=0.8))
                obstacle_box = compute_box_corners(center[0], center[1], theta, ego_length, ego_width)
                # ax_xy.plot(
                #     obstacle_box[:, 0], obstacle_box[:, 1], color='orange', linewidth=1.5, alpha=0.9,
                #     label=tr('橙线：障碍物Box', 'Orange: obstacle box') if i == 0 else None)

            if polygon:
                poly = np.array(polygon, dtype=float)
                if len(poly) >= 2:
                    poly_closed = np.vstack([poly, poly[0]])
                    ax_xy.plot(
                        poly_closed[:, 0], poly_closed[:, 1], color='m', linewidth=1.5, alpha=0.9,
                        label=tr('紫线：障碍物多边形', 'Magenta: obstacle polygon') if i == 0 else None)
                    ax_xy.scatter(
                        poly[:, 0], poly[:, 1], color='m', s=20,
                        label=tr('紫点：障碍物顶点', 'Magenta dots: polygon vertices') if i == 0 else None)
    
    # 绘制车辆方向
    skip = max(1, len(states) // 20)  # 最多显示20个方向箭头
    for i in range(0, len(states), skip):
        dx = 0.5 * np.cos(states[i, 2])
        dy = 0.5 * np.sin(states[i, 2])
        ax_xy.arrow(states[i, 0], states[i, 1], dx, dy,
                    head_width=0.2, head_length=0.1, fc='g', ec='g', alpha=0.6)

    direction_proxy = Line2D(
        [], [], color='g', linewidth=1.5,
        label=tr('绿箭头：车头方向', 'Green arrows: heading direction'))
    # 在轨迹上采样绘制车辆box（由位置和航向决定）
    for i in range(0, len(states), skip):
        ego_box = compute_box_corners(states[i, 0], states[i, 1], states[i, 2], ego_length, ego_width)
        ax_xy.plot(
            ego_box[:, 0], ego_box[:, 1], color='c', linewidth=1.0, alpha=0.35,
            label=tr('青线：自车Box采样', 'Cyan: ego box samples') if i == 0 else None)

    handles_xy, labels_xy = ax_xy.get_legend_handles_labels()
    ax_xy.legend(handles_xy + [direction_proxy],
                 labels_xy + [direction_proxy.get_label()], loc='best')
    ax_xy.set_xlabel(tr('X 位置 (m)', 'X position (m)'))
    ax_xy.set_ylabel(tr('Y 位置 (m)', 'Y position (m)'))
    ax_xy.set_title(tr('车辆轨迹', 'Vehicle trajectory'))
    ax_xy.grid(True)
    ax_xy.axis('equal')
    
    # 2. 控制输入图
    ax_u = plt.subplot(2, 2, 2)
    time = np.arange(len(controls)) * 0.05  # 假设dt=0.05s
    ax_u.plot(time, controls[:, 0], 'r-', linewidth=2,
              label=tr('红实线：加速度', 'Red solid: acceleration'))
    ax_u.axhline(y=2.0, color='r', linestyle='--', alpha=0.5,
                 label=tr('红虚线：加速度上限', 'Red dashed: accel upper limit'))
    ax_u.axhline(y=-2.0, color='r', linestyle='-.', alpha=0.6,
                 label=tr('红点划线：加速度下限', 'Red dash-dot: accel lower limit'))
    ax_u.set_xlabel(tr('时间 (s)', 'Time (s)'))
    ax_u.set_ylabel(tr('加速度 (m/s²)', 'Acceleration (m/s^2)'))
    ax_u.set_title(tr('控制输入', 'Control inputs'))
    ax_u.grid(True)
    
    # 创建第二个y轴用于转向角
    ax2 = ax_u.twinx()
    ax2.plot(time, controls[:, 1], 'b-', linewidth=2,
             label=tr('蓝实线：转向角', 'Blue solid: steering angle'))
    ax2.axhline(y=0.6, color='b', linestyle='--', alpha=0.5,
                label=tr('蓝虚线：转向上限', 'Blue dashed: steering upper limit'))
    ax2.axhline(y=-0.6, color='b', linestyle='-.', alpha=0.6,
                label=tr('蓝点划线：转向下限', 'Blue dash-dot: steering lower limit'))
    ax2.set_ylabel(tr('转向角 (rad)', 'Steering angle (rad)'))
    
    # 合并图例
    lines1, labels1 = ax_u.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax_u.legend(lines1 + lines2, labels1 + labels2, loc='best', fontsize=9)
    
    # 3. 状态变量图
    ax_pos = plt.subplot(2, 2, 3)
    time = np.arange(len(states)) * 0.05  # 假设dt=0.05s
    ax_pos.plot(time, states[:, 0], 'r-', linewidth=2,
                label=tr('红线：X 位置', 'Red: X position'))
    ax_pos.plot(time, states[:, 1], 'g-', linewidth=2,
                label=tr('绿线：Y 位置', 'Green: Y position'))
    ax_pos.set_xlabel(tr('时间 (s)', 'Time (s)'))
    ax_pos.set_ylabel(tr('位置 (m)', 'Position (m)'))
    ax_pos.set_title(tr('位置随时间变化', 'Position vs time'))
    ax_pos.legend(loc='best')
    ax_pos.grid(True)
    
    # 4. 速度和航向角图
    ax_v = plt.subplot(2, 2, 4)
    ax_v.plot(time, states[:, 3], 'b-', linewidth=2,
              label=tr('蓝线：速度', 'Blue: speed'))
    ax_v.set_xlabel(tr('时间 (s)', 'Time (s)'))
    ax_v.set_ylabel(tr('速度 (m/s)', 'Speed (m/s)'), color='b')
    ax_v.tick_params(axis='y', labelcolor='b')
    ax_v.set_title(tr('速度和航向角随时间变化', 'Speed and heading vs time'))
    ax_v.grid(True)
    
    # 创建第二个y轴用于航向角
    ax4_2 = ax_v.twinx()
    ax4_2.plot(time, np.degrees(states[:, 2]), 'r-', linewidth=2,
               label=tr('红线：航向角', 'Red: heading'))
    ax4_2.set_ylabel(tr('航向角 (度)', 'Heading (deg)'), color='r')
    ax4_2.tick_params(axis='y', labelcolor='r')
    
    # 合并图例
    lines1, labels1 = ax_v.get_legend_handles_labels()
    lines2, labels2 = ax4_2.get_legend_handles_labels()
    ax_v.legend(lines1 + lines2, labels1 + labels2, loc='best')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"{tr('轨迹图已保存到', 'Trajectory figure saved to')}: {save_path}")
    
    plt.show()

def visualize_cost_convergence(iterations, costs, save_path=None):
    """
    可视化成本收敛过程
    """
    if not iterations or not costs:
        print(tr("警告: 没有成本收敛数据", "Warning: no cost convergence data"))
        return

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    axes[0].plot(iterations, costs, 'bo-', linewidth=2, markersize=6,
                 label=tr('蓝线：总成本', 'Blue: total cost'))
    axes[0].set_xlabel(tr('迭代次数', 'Iteration'))
    axes[0].set_ylabel(tr('总成本', 'Total cost'))
    axes[0].set_title(tr('iLQR成本收敛曲线', 'iLQR cost convergence'))
    axes[0].grid(True)
    axes[0].legend(loc='best')

    axes[1].semilogy(iterations, costs, 'ro-', linewidth=2, markersize=6,
                     label=tr('红线：总成本（对数）', 'Red: total cost (log)'))
    axes[1].set_xlabel(tr('迭代次数', 'Iteration'))
    axes[1].set_ylabel(tr('总成本（对数坐标）', 'Total cost (log scale)'))
    axes[1].set_title(tr('iLQR成本收敛曲线（对数坐标）',
                         'iLQR cost convergence (log scale)'))
    axes[1].grid(True)
    axes[1].legend(loc='best')

    fig.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"{tr('成本收敛图已保存到', 'Cost convergence figure saved to')}: {save_path}")

    plt.show()

def main():
    parser = argparse.ArgumentParser(description='iLQR可视化工具')
    parser.add_argument('--trajectory', '-t', type=str, 
                        help='轨迹文件路径')
    parser.add_argument('--log', '-l', type=str,
                        help='日志文件路径')
    parser.add_argument('--target', nargs='*',
                        help='目标状态，支持 "x y theta v" 或 "x,y,theta,v"')
    parser.add_argument('--save', '-s', type=str,
                        help='保存图像路径 (不包含扩展名)')
    parser.add_argument('--show_cost', action='store_true',
                        help='显示成本收敛曲线')
    parser.add_argument('--ego_length', type=float, default=15.0,
                        help='车辆/障碍物box长度（默认15.0）')
    parser.add_argument('--ego_width', type=float, default=3.2,
                        help='车辆/障碍物box宽度（默认3.2）')
    
    args = parser.parse_args()
    
    try:
        target = parse_target_args(args.target)
    except ValueError as e:
        parser.error(str(e))

    # 解析轨迹文件
    states, controls, total_cost, obstacles = parse_trajectory_file(args.trajectory)
    
    if states is not None:
        print(f"成功加载轨迹数据，共 {len(states)} 个时间步")
        print(f"总成本: {total_cost:.6f}")
        print(f"起点: [{states[0, 0]:.2f}, {states[0, 1]:.2f}, {np.degrees(states[0, 2]):.2f}°, {states[0, 3]:.2f}]")
        print(f"终点: [{states[-1, 0]:.2f}, {states[-1, 1]:.2f}, {np.degrees(states[-1, 2]):.2f}°, {states[-1, 3]:.2f}]")
        print(f"障碍物数量: {len(obstacles)}")
        
        # 可视化轨迹
        save_path = f"{args.save}_trajectory.png" if args.save else None
        visualize_trajectory(states, controls, target, obstacles, args.ego_length,
                             args.ego_width, save_path)
    
    # 解析并可视化成本收敛
    if args.show_cost:
        if not args.log:
            print(tr("警告: 启用 --show_cost 时请提供 --log 日志文件路径",
                     "Warning: please provide --log when --show_cost is enabled"))
            return
        iterations, costs = parse_log_file(args.log)
        if iterations and costs:
            print(f"成功加载成本数据，共 {len(iterations)} 次迭代")
            save_path = f"{args.save}_cost.png" if args.save else None
            visualize_cost_convergence(iterations, costs, save_path)

if __name__ == "__main__":
    main()
