import numpy as np
import copy
from enum import Enum, IntEnum, auto

# 定义PlannerStatus枚举类型
class PlannerStatus(Enum):
    STANDBY = auto()  # 待机
    COVERAGE_SEARCH = auto()  # 覆盖搜索
    NEARST_UNVISITED_SEARCH = auto()  # 最近未访问搜索
    FOUND = auto()  # 找到目标
    NOT_FOUND = auto()  # 未找到目标

# 定义HeuristicType枚举类型
class HeuristicType(Enum):
    MANHATTAN = auto()  # 曼哈顿距离启发式
    CHEBYSHEV = auto()  # 切比雪夫距离启发式
    VERTICAL = auto()  # 垂直启发式
    HORIZONTAL = auto()  # 水平启发式

# 定义CoveragePlanner类
class CoveragePlanner():

    def __init__(self, map_open):
        self.map_grid = map_open  # 地图网格

        # 在x和y轴上的可能移动方式
        self.movement = [[-1,  0],  # 上
                         [0, -1],    # 左
                         [1,  0],    # 下
                         [0,  1]]    # 右

        # 可读的移动描述['上', '左', '下', '右']
        self.movement_name = ['^', '<', 'v', '>']

        # 机器人可能执行的动作
        self.action = [-1, 0, 1, 2]
        self.action_name = ['R', '#', 'L', 'B']  # 右、前进、左、后退
        self.action_cost = [.2, .1, .2, .4]

        # A*算法的移动成本
        self.a_star_movement_cost = [1, 1, 1, 1]

        # 当前位置 [x, y, 方向 (默认 = 0)]
        self.current_pos = self.get_start_position()

        # 轨迹点的列表
        self.current_trajectory = []
        self.current_trajectory_annotations = []

        # 累积访问过的地图位置的网格
        self.coverage_grid = np.copy(map_open)

        # 有限状态机变量
        self.state_ = PlannerStatus.STANDBY  # 初始状态为待机

        # 各种搜索算法的启发式类型
        self.a_star_heuristic = HeuristicType.MANHATTAN
        self.cp_heuristic = HeuristicType.VERTICAL

        self.debug_level = -1  # 调试级别，默认为-1（不显示调试信息）

    # 设置调试级别
    # 决定终端中要显示多少信息
    def set_debug_level(self, level):
        self.debug_level = level

    # 执行路径规划
    def compute(self):
        self.printd("compute", "{}".format(self.state_.name), 1)
        while self.compute_non_blocking():
            pass
        return self.state_

    # 处理路径规划的有限状态机
    def compute_non_blocking(self):
        self.printd("compute_non_blocking", "{}".format(self.state_.name), 1)
        searching = False

        # 根据self.state_属性开始FSM状态机
        if self.state_ == PlannerStatus.COVERAGE_SEARCH:

            # 使用coverage_search算法进行搜索
            heuristic = self.create_heuristic(
                self.current_pos, self.cp_heuristic)
            res = self.coverage_search(self.current_pos, heuristic)

            # 更新当前位置到最终搜索位置
            self.current_pos = [res[1][-1][1], res[1][-1][2], res[1][-1][3]]

            self.append_trajectory(res[1], "CS")

            # 更新当前coverage_grid
            self.coverage_grid = res[2]

            # 检查路径是否成功找到。如果没有，则尝试找到最近的未访问位置
            if res[0]:
                self.state_ = PlannerStatus.FOUND
                self.current_trajectory[-1][6] = PlannerStatus.FOUND
            else:
                self.state_ = PlannerStatus.NEARST_UNVISITED_SEARCH
                searching = True

        elif self.state_ == PlannerStatus.NEARST_UNVISITED_SEARCH:

            # 使用a_star_search_closest_unvisited算法进行搜索
            heuristic = self.create_heuristic(
                self.current_pos, self.a_star_heuristic)
            res = self.a_star_search_closest_unvisited(
                self.current_pos, heuristic)

            # 如果找到路径
            if res[0]:
                # 更新当前位置到最终搜索位置
                self.current_pos = [res[1][-1][1],
                                    res[1][-1][2], res[1][-1][3]]

                self.append_trajectory(res[1], "A*")

                # 设置FSM以再次进行覆盖搜索
                self.state_ = PlannerStatus.COVERAGE_SEARCH
                searching = True

            # 如果找不到路径，就结束搜索
            else:
                self.state_ = PlannerStatus.NOT_FOUND
                if len(self.current_trajectory) > 0:
                    self.current_trajectory[-1][6] = PlannerStatus.NOT_FOUND

        else:
            self.printd("compute_non_blocking",
                        "给定的状态无效，停止FSM", 0)

        return searching

    # 重新开始初始位置，覆盖网格和轨迹列表，并准备开始搜索
    def start(self, initial_orientation=0, a_star_heuristic=None, cp_heuristic=None):

        # 将当前位置设置为给定地图的起始位置
        self.current_pos = self.get_start_position(
            orientation=initial_orientation)

        self.coverage_grid = np.copy(self.map_grid)
        self.current_trajectory = []
        self.current_trajectory_annotations = []

        if cp_heuristic is not None:
            self.cp_heuristic = cp_heuristic
        if a_star_heuristic is not None:
            self.a_star_heuristic = a_star_heuristic

        self.state_ = PlannerStatus.COVERAGE_SEARCH
        self.printd("start", "搜索设置为从{}开始，轨迹和覆盖网格已清除".format(
            self.current_pos), debug_level=1)

    # 使用coverage_search算法查找路径
    def coverage_search(self, initial_pos, heuristic):
        # 创建已访问坐标的参考网格
        closed = np.copy(self.coverage_grid)
        closed[initial_pos[0]][initial_pos[1]] = 1

        if self.debug_level > 1:
            self.printd("coverage_search",
                        "初始已关闭网格:", 2)
            print(closed)

        x = initial_pos[0]
        y = initial_pos[1]
        o = initial_pos[2]
        v = 0

        # 将初始坐标填充到迭代列表中
        trajectory = [[v, x, y, o, None, None, self.state_]]

        complete_coverage = False
        resign = False

        while not complete_coverage and not resign:

            if self.check_full_coverage(self.map_grid, closed):
                self.printd("coverage_search", "完全覆盖", 2)
                complete_coverage = True

            else:
                # 获取上一个访问的坐标信息
                v = trajectory[-1][0]
                x = trajectory[-1][1]
                y = trajectory[-1][2]
                o = trajectory[-1][3]

                # [累积成本, x坐标, y坐标, 方向, 执行的动作, 下一个动作]
                possible_next_coords = []

                # 计算可能的下一个坐标
                for a in range(len(self.action)):
                    o2 = (self.action[a]+o) % len(self.movement)
                    x2 = x + self.movement[o2][0]
                    y2 = y + self.movement[o2][1]

                    # 检查是否超出地图边界
                    if x2 >= 0 and x2 < len(self.map_grid) and y2 >= 0 and y2 < len(self.map_grid[0]):
                        # 检查此位置是否已访问或是否为可访问的位置
                        if closed[x2][y2] == 0 and self.map_grid[x2][y2] == 0:
                            # 计算累积成本：当前累积成本 + 动作成本 + 给定位置的启发成本
                            v2 = v + self.action_cost[a] + heuristic[x2][y2]
                            possible_next_coords.append(
                                [v2, x2, y2, o2, a, None, self.state_])

                # 如果没有可能的下一个位置，停止搜索
                if len(possible_next_coords) == 0:
                    resign = True
                    self.printd("coverage_search",
                                "找不到下一个未访问的坐标", 2)

                # 否则使用具有最低成本的下一个位置更新轨迹列表
                else:
                    # 按总成本排序
                    possible_next_coords.sort(key=lambda x: x[0])

                    # 更新最后轨迹的下一个动作
                    trajectory[-1][5] = possible_next_coords[0][4]

                    # 将具有最低成本的possible_next_coords添加到轨迹列表
                    trajectory.append(possible_next_coords[0])

                    # 将已选择的possible_next_coords位置标记为已访问
                    closed[possible_next_coords[0][1]
                           ][possible_next_coords[0][2]] = 1

        if self.debug_level > 1:
            self.printd("coverage_search", "启发式：", 2)
            print(heuristic)

            self.printd("coverage_search", "已关闭：", 2)
            print(closed)

            self.printd("coverage_search", "策略：", 2)
            self.print_policy_map(trajectory, [])

            self.printd("coverage_search", "轨迹：", 2)
            self.print_trajectory(trajectory)

        total_cost = self.calculate_trajectory_cost(trajectory)
        total_steps = len(trajectory)-1

        self.printd("coverage_search", "找到: {}, 总步数: {}, 总成本: {}".format(
            not resign, total_steps, total_cost), 1)

        # 打包标准响应
        # 轨迹：[值, x, y, 方向, 执行的动作, 下一个动作, 当前状态_]
        # res: [成功？, 轨迹, 最终覆盖网格, 总动作成本, 总步数]
        res = [not resign, trajectory, closed, total_cost, total_steps]

        return res

    # 使用A*搜索算法找到初始坐标和目标坐标之间的最短路径
    def a_star_search_closest_unvisited(self, initial_pos, heuristic):

        # 创建一个已访问位置的参考网格
        closed = np.zeros_like(self.map_grid)
        closed[initial_pos[0]][initial_pos[1]] = 1

        if self.debug_level > 1:
            self.printd("a_star_search_closest_unvisited",
                        "初始已关闭网格：", 2)
            print(closed)

        # A*访问位置的移动方向
        orientation = np.full(
            (np.size(self.map_grid, 0), np.size(self.map_grid, 1)), -1)

        # 将给定的A*初始位置与其关联的成本添加到“open”列表中
        # “open”是要扩展的有效位置列表：[[f, g, x, y]]
        # g：累积a*移动成本（以0成本开始）
        # f：总成本= a*移动成本+给定位置的启发成本
        # x，y：给定位置
        x = initial_pos[0]
        y = initial_pos[1]
        g = 0
        f = g + heuristic[x][y]
        open = [[f, g, x, y]]

        found = False  # 是否找到了未访问的位置
        resign = False  # 如果我们找不到扩展，则设置标志

        while not found and not resign:
            self.printd("a_star_search_closest_unvisited",
                        " open：{}".format(open), 2)

            # 如果没有更多要扩展的位置，则未找到未访问的位置，然后放弃
            if len(open) == 0:
                resign = True
                self.printd("a_star_search_closest_unvisited",
                            " 未找到路径", 2)

            # 否则再次扩展搜索
            else:

                # 按总成本的降序对位置列表进行排序并反转它，以弹出具有最低总成本的元素
                open.sort(key=lambda x: x[0])  # +heuristic[x[1]][x[2]])
                open.reverse()
                next = open.pop()

                # 更新当前搜索的x，y，g
                x = next[2]
                y = next[3]
                g = next[1]

                # 检查是否找到了未访问的位置
                if self.coverage_grid[x][y] == 0:
                    found = True
                else:
                    # 计算可能的下一个坐标
                    for i in range(len(self.movement)):
                        x_next = x + self.movement[i][0]
                        y_next = y + self.movement[i][1]

                        # 检查是否超出地图边界
                        if x_next >= 0 and x_next < len(self.map_grid) and y_next >= 0 and y_next < len(self.map_grid[0]):
                            # 检查此位置是否已访问或是否为可访问的位置
                            if closed[x_next][y_next] == 0 and self.map_grid[x_next][y_next] == 0:
                                g2 = g + self.a_star_movement_cost[i]
                                f = g2 + heuristic[x_next][y_next]
                                open.append([f, g2, x_next, y_next])
                                closed[x_next][y_next] = 1
                                orientation[x_next][y_next] = i

        # 初始化轨迹
        trajectory = []

        # 如果找到路径，则构建轨迹。
        # 此时x和y代表搜索的最后一个位置，即未访问的位置
        if found:

            # 将最后一个位置添加到轨迹列表中，两个操作均为None（稍后将设置）
            trajectory = [
                [0, x, y, orientation[x][y], None, None, self.state_]]

            # 将初始方向添加到方向矩阵中
            orientation[initial_pos[0]][initial_pos[1]] = initial_pos[2]

            # 从找到的未访问的位置向后移动，直到到达A*的初始位置
            # 此下标0表示该变量是前身位置的，因为
            # 这个过程从A*的最终位置开始，到A*的初始位置
            while (x != initial_pos[0] or y != initial_pos[1]):
                # 计算前身位置
                x0 = x - self.movement[orientation[x][y]][0]
                y0 = y - self.movement[orientation[x][y]][1]
                # 前身方向是在方向矩阵上的方向
                o0 = orientation[x0][y0]
                # 前身操作将在下一次迭代中设置（它是它之前的下一个操作）
                a0 = None

                # 计算从前身位置到当前迭代位置所需的操作索引
                a = (trajectory[-1][3]-o0 + 1) % len(self.action)

                # 更新后继位置的“action_performed_to_get_here”以当前的下一个动作
                trajectory[-1][4] = a

                # 将前身位置和操作添加到轨迹列表中
                trajectory.append([0, x0, y0, o0, a0, a, self.state_])

                # 更新x和y以进行下一次迭代
                x = x0
                y = y0

            trajectory.reverse()

        if self.debug_level > 1:
            self.printd("a_star_search_closest_unvisited", "启发式：", 2)
            print(heuristic)

            self.printd("a_star_search_closest_unvisited", "已关闭：", 2)
            print(closed)

            self.printd("a_star_search_closest_unvisited", "策略：", 2)
            self.print_policy_map(trajectory, orientation)

            self.printd("a_star_search_closest_unvisited", "轨迹：", 2)
            self.print_trajectory(trajectory)

        total_cost = self.calculate_trajectory_cost(trajectory)
        total_steps = len(trajectory)-1

        self.printd("a_star_search_closest_unvisited", "找到: {}, 总步数: {}, 总成本: {}".format(
            found, total_steps, total_cost), 1)

        # 打包标准响应
        # 轨迹：[值, x, y, 方向, 执行的动作, 下一个动作, 当前状态_]
        # res: [成功？, 轨迹, 最终覆盖网格, 总动作成本, 总步数]
        res = [found, trajectory, closed, total_cost, total_steps]

        return res

    # 合并给定的两个网格，并返回True，如果所有可访问的位置都已访问
    def check_full_coverage(self, grid, closed):
        return np.all(np.copy(grid)+np.copy(closed))

    # 返回给定目标点的曼哈顿启发式
    def create_manhattan_heuristic(self, target_point):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                heuristic[x][y] = abs(x-target_point[0])+abs(y-target_point[1])
        return heuristic

    # 返回给定目标点的切比雪夫启发式
    def create_chebyshev_heuristic(self, target_point):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                heuristic[x][y] = max(
                    abs(x-target_point[0]), abs(y-target_point[1]))
        return heuristic

    # 返回给定目标点的水平启发式
    def create_horizontal_heuristic(self, target_point):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                heuristic[x][y] = abs(x-target_point[0])
        return heuristic

    # 返回给定目标点的垂直启发式
    def create_vertical_heuristic(self, target_point):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                heuristic[x][y] = abs(y-target_point[1])
        return heuristic

    # 返回给定目标点和启发式类型的启发式
    def create_heuristic(self, target_point, heuristic_type):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                if heuristic_type == HeuristicType.MANHATTAN:
                    heuristic[x][y] = abs(
                        x-target_point[0]) + abs(y-target_point[1])
                elif heuristic_type == HeuristicType.CHEBYSHEV:
                    heuristic[x][y] = max(
                        abs(x-target_point[0]), abs(y-target_point[1]))
                elif heuristic_type == HeuristicType.HORIZONTAL:
                    heuristic[x][y] = abs(x-target_point[0])
                elif heuristic_type == HeuristicType.VERTICAL:
                    heuristic[x][y] = abs(y-target_point[1])
        return heuristic

    # 返回当前地图网格的初始x、y和方向
    def get_start_position(self, orientation=0):
        for x in range(len(self.map_grid)):
            for y in range(len(self.map_grid[0])):
                if(self.map_grid[x][y] == 2):
                    return [x, y, orientation]

    # 将给定轨迹附加到主轨迹
    def append_trajectory(self, new_trajectory, algorithm_ref):
        # 如果已经有轨迹位置，则通过连接动作来删除重复位置，并在新轨迹的第一个位置列表末尾添加一个特殊的观测
        if len(self.current_trajectory) > 0 and len(new_trajectory) > 0:
            # 由于每次搜索只依赖于当前位置，因此必须将累积轨迹的最后一个元素的“action_performed_to_get_here”复制到新轨迹的第一个元素中。
            new_trajectory[0][4] = self.current_trajectory[-1][4]

            # 添加一个特殊标注以显示在策略地图上
            self.current_trajectory_annotations.append(
                [new_trajectory[0][1], new_trajectory[0][2], algorithm_ref])

            # 移除重复的位置
            self.current_trajectory.pop()

        # 将计算得到的路径添加到轨迹列表中
        for t in new_trajectory:
            self.current_trajectory.append(t)

    # 计算轨迹的总成本
    def calculate_trajectory_cost(self, trajectory):
        cost = 0

        # 将每个步骤的动作成本相加
        for t in trajectory:
            if t[5] is not None:
                cost += self.action_cost[t[5]]
        return cost

    # 返回仅包含轨迹xy的numpy数组
    def get_xy_trajectory(self, trajectory):
        if type(trajectory) == list:
            if type(trajectory[0]) == list:
                return [t[1:3] for t in trajectory]
            return trajectory[1:3]
        return []

    # 返回搜索结果：[found?, total_steps, total_cost, trajectory, xy_trajectory]
    def result(self):
        found = self.state_ == PlannerStatus.FOUND
        total_steps = len(self.current_trajectory)-1
        total_cost = self.calculate_trajectory_cost(self.current_trajectory)
        xy_trajectory = self.get_xy_trajectory(self.current_trajectory)

        res = [found, total_steps, total_cost,
               self.current_trajectory, xy_trajectory]
        return res

    # 打印搜索结果的摘要
    def show_results(self):
        self.printd("show_results",
                    "展示当前的搜索结果：\n")
        self.printd("show_results",
                    "最终状态: {}".format(self.state_.name))
        # 最后一个元素只指向最后一个轨迹位置，它不是已完成的步骤。
        self.printd("show_results", "总步数: {}".format(
            len(self.current_trajectory)-1))
        self.printd("show_results", "总成本: {:.2f}".format(
            self.calculate_trajectory_cost(self.current_trajectory)))
        if self.debug_level > 0:
            self.print_trajectory(self.current_trajectory)
        self.print_policy_map()

    # 打印轨迹数据
    def print_trajectory(self, trajectory):
        self.printd("print_trajectory", "轨迹数据:\n")
        print("{}\t{}\t{}\t{}\t{}\t{}".format(
            "l_cost", "x", "y", "orient.", "act_in", "act_next"))
        for t in trajectory:
            print("{:.2f}\t{}\t{}\t{}\t{}\t{}\t{}".format(
                t[0], t[1], t[2], t[3], t[4], t[5], t[6].name))

    # 打印具有标准列宽的给定地图网格
    def print_map(self, m):
        for row in m:
            s = "["
            for i in range(len(m[0])):
                if type(row[i]) is str:
                    s += row[i]
                else:
                    s += "{:.1f}".format(row[i])
                if i is not (len(m[0])-1):
                    s += "\t,"
                else:
                    s += "\t]"
            print(s)

    # 计算并打印基于轨迹列表的当前策略地图
    def print_policy_map(self, trajectory=None, trajectory_annotations=None):
        policy = [[" " for row in range(len(self.map_grid[0]))]
                  for col in range(len(self.map_grid))]

        # 放置障碍物的参考
        for col in range(len(self.map_grid[0])):
            for row in range(len(self.map_grid)):
                if self.map_grid[row][col] == 1:
                    policy[row][col] = "XXXXXX"

        if trajectory == None:
            trajectory = self.current_trajectory

        if trajectory_annotations == None:
            trajectory_annotations = self.current_trajectory_annotations

        # 在每个位置放置下一个动作名称
        for t in trajectory:
            if t[5] is not None:
                policy[t[1]][t[2]] += self.action_name[t[5]]

        # 在地图上放置注释
        trajectory_annotations.append(
            [trajectory[0][1], trajectory[0][2], "STA"])
        trajectory_annotations.append(
            [trajectory[-1][1], trajectory[-1][2], "END"])

        for t in trajectory_annotations:
            policy[t[0]][t[1]] += "@"+t[2]

        self.printd("print_policy_map", "策略地图:\n")
        self.print_map(policy)

    # 带有标准化打印结构的打印辅助函数
    # [function_name] message
    def printd(self, f, m, debug_level=0):
        if debug_level <= self.debug_level:
            print("["+f+"] "+m)
