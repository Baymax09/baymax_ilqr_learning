#!/bin/bash

# iLQR运行和可视化脚本
# 此脚本将运行iLQR求解器并可视化结果

# 设置默认参数
DT=${DT:-0.05}
WHEELBASE=${WHEELBASE:-2.5}
HORIZON=${HORIZON:-200}
X0=${X0:-"[0.0, 0.0, 0.0, 0.0]"}
TARGET=${TARGET:-"[10.0, 10.0, 0.0, 0.0]"}
SAVE_TRAJECTORY=${SAVE_TRAJECTORY:-true}
TRAJECTORY_FILE=${TRAJECTORY_FILE:-"ilqr_trajectory.txt"}
LOG_FILE=${LOG_FILE:-"ilqr_log.txt"}
# 障碍物参数默认不在脚本里强制覆盖，保持使用 ilqr_node 的 declare_parameter 默认值
OBSTACLE_X=${OBSTACLE_X:-}
OBSTACLE_Y=${OBSTACLE_Y:-}
OBSTACLE_RADIUS=${OBSTACLE_RADIUS:-}
OBSTACLE_WEIGHT=${OBSTACLE_WEIGHT:-}
OBSTACLE_VERTICES=${OBSTACLE_VERTICES:-}
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
WORKSPACE_SETUP="$SCRIPT_DIR/install/setup.bash"
ROS_LOG_DIR=${ROS_LOG_DIR:-"$SCRIPT_DIR/ilqr_results/ros_logs"}
X0_COMPACT=${X0// /}
TARGET_COMPACT=${TARGET// /}
TARGET_ARGS=${TARGET_COMPACT//[\[\],]/ }
export ROS_LOG_DIR

# 创建输出目录
mkdir -p ilqr_results
mkdir -p "$ROS_LOG_DIR"

if [ ! -f "$WORKSPACE_SETUP" ]; then
    echo "错误: 未找到工作区环境文件 $WORKSPACE_SETUP"
    echo "请先在工作区根目录执行 colcon build"
    exit 1
fi

source "$WORKSPACE_SETUP"

# 运行iLQR求解器
echo "运行iLQR求解器..."
echo "参数: dt=$DT, wheelbase=$WHEELBASE, horizon=$HORIZON"
echo "初始状态: $X0_COMPACT"
echo "目标状态: $TARGET_COMPACT"
if [ -n "$OBSTACLE_X$OBSTACLE_Y$OBSTACLE_RADIUS$OBSTACLE_WEIGHT$OBSTACLE_VERTICES" ]; then
    echo "障碍物参数(脚本覆盖): x=${OBSTACLE_X:-<node_default>}, y=${OBSTACLE_Y:-<node_default>}, radius=${OBSTACLE_RADIUS:-<node_default>}, weight=${OBSTACLE_WEIGHT:-<node_default>}, vertices=${OBSTACLE_VERTICES:-<node_default>}"
else
    echo "障碍物参数: 使用 ilqr_node 默认值"
fi
echo "ROS_LOG_DIR: $ROS_LOG_DIR"

OBSTACLE_PARAM_ARGS=()
if [ -n "$OBSTACLE_X" ]; then
    OBSTACLE_PARAM_ARGS+=(-p "obstacle_x:=$OBSTACLE_X")
fi
if [ -n "$OBSTACLE_Y" ]; then
    OBSTACLE_PARAM_ARGS+=(-p "obstacle_y:=$OBSTACLE_Y")
fi
if [ -n "$OBSTACLE_RADIUS" ]; then
    OBSTACLE_PARAM_ARGS+=(-p "obstacle_radius:=$OBSTACLE_RADIUS")
fi
if [ -n "$OBSTACLE_WEIGHT" ]; then
    OBSTACLE_PARAM_ARGS+=(-p "obstacle_weight:=$OBSTACLE_WEIGHT")
fi
if [ -n "$OBSTACLE_VERTICES" ]; then
    OBSTACLE_PARAM_ARGS+=(-p "obstacle_vertices:=$OBSTACLE_VERTICES")
fi

# 重定向输出到日志文件
ros2 run ilqr_learning ilqr_node \
    --ros-args \
    -p dt:=$DT \
    -p wheelbase:=$WHEELBASE \
    -p horizon:=$HORIZON \
    -p x0:="$X0_COMPACT" \
    -p target:="$TARGET_COMPACT" \
    -p save_trajectory:=$SAVE_TRAJECTORY \
    -p trajectory_file:=$TRAJECTORY_FILE \
    "${OBSTACLE_PARAM_ARGS[@]}" \
    2>&1 | tee ilqr_results/$LOG_FILE

# 检查轨迹文件是否生成
if [ -f "$TRAJECTORY_FILE" ]; then
    echo "轨迹文件已生成: $TRAJECTORY_FILE"
    
    # 运行可视化脚本
    echo "运行可视化脚本..."
    python3 visualize_ilqr.py \
        --trajectory "$TRAJECTORY_FILE" \
        --log "ilqr_results/$LOG_FILE" \
        --target $TARGET_ARGS \
        --save ilqr_results/ilqr_result \
        --show_cost
    VIS_EXIT_CODE=$?

    # 移动轨迹文件到结果目录
    mv "$TRAJECTORY_FILE" ilqr_results/

    if [ $VIS_EXIT_CODE -eq 0 ]; then
        echo "结果已保存在 ilqr_results/ 目录中"
    else
        echo "警告: 可视化脚本执行失败，请检查上方报错"
        exit $VIS_EXIT_CODE
    fi
else
    echo "错误: 轨迹文件未生成"
fi
