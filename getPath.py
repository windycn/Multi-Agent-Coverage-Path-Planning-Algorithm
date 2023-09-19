import numpy as np
from PathPlanningCore import CoveragePlanner, HeuristicType, PlannerStatus
from tabulate import tabulate
import os

import matplotlib.pyplot as plt
import matplotlib as mpl

# 指定中文字体的路径
font_path = "C:\\Windows\\Fonts\\simhei.ttf"  # 根据实际路径进行修改
plt.rcParams['font.family'] = 'SimHei'

from matplotlib.patches import Patch
from matplotlib.lines import Line2D

# 覆盖规划器的调试级别
cp_debug_level = 0
# 是否显示每个结果的测试标志
test_show_each_result = False

# 载入地图
def load_map(map_name):
    with open("maps/{}.npy".format(map_name), 'rb') as f:
        return np.load(f)

# 使用matplotlib绘制结果
def plot_map(target_map, trajectory, map_name="map", params_str=""):
    # 从CoveragePlanner到转换动作为定向移动的参考
    movement = [[-1,  0],  # 上
                [0, -1],    # 左
                [1,  0],    # 下
                [0,  1]]    # 右
    action = [-1, 0, 1, 2]

    # 创建一个图形
    fig, ax = plt.subplots()

    # 定义颜色
    start_position_color = 'gold'  # 起始位置颜色
    start_orientation_color = 'deeppink'  # 起始方向颜色
    status_color_ref = {
        PlannerStatus.STANDBY: 'black',
        PlannerStatus.COVERAGE_SEARCH: 'royalblue',
        PlannerStatus.NEARST_UNVISITED_SEARCH: 'darkturquoise',
        PlannerStatus.FOUND: 'mediumseagreen',
        PlannerStatus.NOT_FOUND: 'red'
    }

    cmap = mpl.colors.ListedColormap(
        ['w', 'k', start_position_color, status_color_ref[PlannerStatus.FOUND], status_color_ref[PlannerStatus.NOT_FOUND]])
    norm = mpl.colors.BoundaryNorm([0, 1, 2, 3, 4, 5], cmap.N)

    # 定义状态到cmap引用idx的转换
    status_to_cmap_pos = {
        PlannerStatus.FOUND: 3,
        PlannerStatus.NOT_FOUND: 4
    }

    # 复制原始地图以避免更改
    target_map_ref = np.copy(target_map)

    # 将最后访问的位置的参考添加到地图中，以反映其在地图上的颜色
    target_map_ref[
        trajectory[-1][1]][trajectory[-1][2]] = status_to_cmap_pos[trajectory[-1][6]]

    # 绘制带颜色的地图
    ax.imshow(target_map_ref, interpolation='none', cmap=cmap, norm=norm)

    # 在轨迹的每个动作上绘制箭头
    for i in range(len(trajectory)-1):

        x = trajectory[i][2]
        y = trajectory[i][1]

        # 将动作值添加到当前方向将导致的移动索引
        mov_idx = (trajectory[i][3]+action[trajectory[i][5]]) % len(movement)
        mov = movement[mov_idx]

        # 从参考列表中获取对应的状态颜色
        arrow_color = status_color_ref[trajectory[i][6]]

        # 仅为了改善可视化，将A*箭头略微右移/下移
        if trajectory[i][6] == PlannerStatus.NEARST_UNVISITED_SEARCH:
            # 检查是否为垂直或水平移动
            if mov_idx % 2:
                y -= 0.25
            else:
                x += 0.25

        # 从当前位置到下一个位置添加箭头点
        ax.arrow(x, y, mov[1], mov[0], width=0.1,
                 color=arrow_color, length_includes_head=True)

    # 绘制初始方向
    init_direction = np.array(movement[trajectory[0][3]])/2
    ax.arrow(trajectory[0][2]-init_direction[1]/2, trajectory[0][1]-init_direction[0]/2, init_direction[1], init_direction[0], width=0.1,
             color=start_orientation_color, length_includes_head=True, head_length=0.2)

    # 添加图例
    legend_elements = [
        Line2D([0], [0], color=status_color_ref[PlannerStatus.COVERAGE_SEARCH], lw=1, marker='>',
               markerfacecolor=status_color_ref[PlannerStatus.COVERAGE_SEARCH], label='前进(覆盖搜索)'),
        Line2D([0], [0], color=status_color_ref[PlannerStatus.NEARST_UNVISITED_SEARCH], lw=1, marker='>',
               markerfacecolor=status_color_ref[PlannerStatus.NEARST_UNVISITED_SEARCH], label='迂回(A*搜索)'),
        Line2D([0], [0], color='w', lw=1, marker='>',
               markerfacecolor=start_orientation_color, label='初始方向'),
        Line2D([0], [0], marker='s', color='w', label='起始位置',
               markerfacecolor=start_position_color, markersize=15),
        Line2D([0], [0], marker='s', color='w', label='结束位置',
               markerfacecolor=status_color_ref[trajectory[-1][6]], markersize=15),
        Line2D([0], [0], marker='s', color='w',
               label='障碍物', markerfacecolor='k', markersize=15),
    ]
    ax.legend(handles=legend_elements, bbox_to_anchor=(
        1.025, 1.0), loc='upper left')
    plt.title("覆盖路径规划[{}]\n{}".format(map_name, params_str))
    plt.tight_layout()
    plt.show()
    # 检查目标文件夹是否存在，如果不存在则创建
    if not os.path.exists("output_images"):
        os.makedirs("output_images")
    fig.savefig("output_images/{}.png".format(map_name), bbox_inches='tight')


def plan_coverage_path(maps: list, isprint=True, isconsole=True ,test_show_each_result=False) -> list:
    """
    覆盖路径规划算法生成函数

    :param maps: (map_name_list) 输入已有的map(npy)格式数据文件名；
    :param isprint: (默认为True) 是否输出图示；
    :param isconsole:  (默认为True) 是否控制台打印信息；
    :param test_show_each_result: (默认为False) 是否显示每个结果的测试标志；
    :return: best_trajectory_list: 最好的路径列表

    {
        "map_name": 地图文件名,

        "start_pos": 起始位置,

        "end_pos": 结束位置,

        "start_orientation": 初始方向 ['^', '<', 'v', '>'],

        "start_orientation_code": 初始方向代码 [0, 1, 2, 3],

        "coverage_path_Heuristic": 启发式算法名称（MANHATTAN曼哈顿距离；CHEBYSHEV切比雪夫距离；VERTICAL垂直启发式；HORIZONTAL水平启发式,

        "Path_point_list": 路径点列表 ([row_id, column_id]),

        "Cost": 总代价,

        "Steps": 总步长,

        "policy_map": 策略地图
    }
    """
    # 为每个地图动态计算最佳覆盖启发式的列表
    cp_heuristics = [HeuristicType.VERTICAL,
                     HeuristicType.HORIZONTAL, HeuristicType.CHEBYSHEV, HeuristicType.MANHATTAN]
    orientations = [0, 1, 2, 3]
    best_trajectory_list = []
    for map_name in maps:
        compare_tb = []

        target_map = load_map(map_name)
        cp = CoveragePlanner(target_map)
        cp.set_debug_level(cp_debug_level)

        # 对每个方向和每个启发式进行迭代
        for heuristic in cp_heuristics:
            for orientation in orientations:
                if test_show_each_result:
                    print("\n\n迭代[地图：{}，cp：{}，初始方向：{}]".format(
                        map_name, heuristic.name, orientation))

                cp.start(initial_orientation=orientation, cp_heuristic=heuristic)
                cp.compute()

                if test_show_each_result:
                    cp.show_results()

                res = [heuristic.name, orientation]
                res.extend(cp.result())
                compare_tb.append(res)

        # 按步数排序
        compare_tb.sort(key=lambda x: (x[3], x[4]))

        # 显示结果
        if isconsole:
            print("测试的地图：{}".format(map_name))

        # 打印给定地图的结果摘要
        summary = [row[0:5] for row in compare_tb]
        for row in summary:
            # 格式化成2位小数的成本
            row[4] = "{:.2f}".format(row[4])
            # 将移动索引转换为移动名称
            row[1] = cp.movement_name[row[1]]

        compare_tb_headers = ["启发式",
                              "初始方向", "找到?", "步数", "成本"]
        summary_tb = tabulate(summary, compare_tb_headers,
                              tablefmt="pretty", floatfmt=".2f")
        if isconsole:
            print(summary_tb)

        # 打印最佳覆盖规划器的策略地图
        if isconsole:
            cp.print_policy_map(trajectory=compare_tb[0][5], trajectory_annotations=[])

        # 绘制完整的轨迹地图
        if isprint:
            plot_map(target_map, compare_tb[0][5], map_name=map_name,
                     params_str="启发式:{}, 初始方向: {}".format(compare_tb[0][0], cp.movement_name[compare_tb[0][1]]))

        # 打印最佳路径
        if isconsole:
            print("\n最佳路径的坐标列表：[地图：{}，初始方向：{} ({})，覆盖路径启发式：{}]".format(
                map_name, cp.movement_name[compare_tb[0][1]], compare_tb[0][1], compare_tb[0][0]))
            print(compare_tb[0][6])
            print("\n\n")

        # 返回信息
        best_trajectory_list.append({
            "map_name": map_name,
            "start_pos": compare_tb[0][6][0],
            "end_pos": compare_tb[0][6][-1],
            "start_orientation": cp.movement_name[compare_tb[0][1]],
            "start_orientation_code": compare_tb[0][1],
            "coverage_path_Heuristic": compare_tb[0][0],
            "Path_point_list": compare_tb[0][6],
            "Cost": summary[0][-1],
            "Steps": summary[0][-2],
            "policy_map": compare_tb[0][5]
        })

    return best_trajectory_list

# 测试
if __name__ == "__main__":
    # 载入地图
    maps = ["map1", "map2", "map3", "map4"]
    best_trajectory_list = plan_coverage_path(maps, False, False, False)
    print(best_trajectory_list)