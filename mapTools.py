import numpy as np
import random
import copy
import os

def gen_base_map(rows=16, cols=19, obstacle_size=2):
    '''
    生成基础的数组地图，可以手动调整障碍后调用map2np生成持久储存使用的npy地图

    :param rows: 行数 (空格为0)
    :param cols: 列数 (空格为0)
    :param obstacle_size: 障碍尺寸 (障碍默认2X2大小，为1)
    :return: grid: 生成的map数组
    '''
    # 创建一个二维数组，初始值为0
    grid = np.zeros((rows, cols), dtype=int)

    # 在边界周围的一圈设置为0
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 0

    # 随机放置大障碍物，确保周围一圈都是0
    for i in range(1, rows - 1, obstacle_size + 1):
        for j in range(1, cols - 1, obstacle_size + 1):
            if grid[i, j] == 0:
                for x in range(obstacle_size):
                    for y in range(obstacle_size):
                        if i + x < rows and j + y < cols:
                            grid[i + x, j + y] = 1

    return grid


def random_obstacle_map(rows=10, cols=12):
    '''
    生成随机障碍地图

    :param rows: 行数 (空格为0)
    :param cols: 列数 (空格为0)
    :return: grid: 生成的map数组
    '''
    # 创建一个二维数组，初始值为0
    grid = np.zeros((rows, cols), dtype=int)

    # 在边界周围的一圈设置为0
    grid[0, :] = grid[-1, :] = grid[:, 0] = grid[:, -1] = 0

    for i in range(2, rows - 2, 2):
        for j in range(2, cols - 2, 2):
            if grid[i, j] == 0:
                if random.choice([True, False]):
                    # 横向障碍
                    grid[i, j] = grid[i, j + 1] = 1
                else:
                    # 纵向障碍
                    grid[i, j] = grid[i + 1, j] = 1

    return grid

def randomStartPoint(input_map: list, startpoint=1):
    '''
    随机生成起始点 (置为2) 加入到地图中(随机四周放点，四周所有的[0][0]和首行[0]与尾行[-1])

    :param input_map: 输入的地图
    :param startpoint: 起始点数量，默认为1（一次规划只能规划最开始的起始点），划分后可以进行多个
    :return: new_map: 含随机起始点的新地图
    '''
    # 使用深拷贝创建副本
    print(input_map)
    input_map = copy.deepcopy(input_map)
    # 定义地图的行数和列数
    map_row = len(input_map)
    map_columns = len(input_map[0])

    # 随机四周放点，四周所有的[0][0]和首行[0]与尾行[-1]
    possible_start_positions = [(0, 0)] + [(0, i) for i in range(map_columns)] + [(map_row - 1, 0)] + [(map_row - 1, i) for i in range(map_columns)]

    # 随机选择不重复的起始位置
    selected_start_positions = random.sample(possible_start_positions, startpoint)

    # 在地图上标记起始点（起始点为2）
    for pos in selected_start_positions:
        input_map[pos[0]][pos[1]] = 2

    print("含起始点({})的地图：".format(selected_start_positions))
    # 打印包含起始点的地图
    for row in input_map:
        print(row)

    return input_map


def map2np(maps: list, map_name_list: list):
    '''
    将数组地图转换成持久储存使用的npy地图

    :param maps: 地图数据列表 (嵌套数组)
    :param map_name_list: 地图名字列表 (也是保存的文件名maps/map_name.npy)
    :return: None
    '''
    # 检查目标文件夹是否存在，如果不存在则创建
    if not os.path.exists("maps"):
        os.makedirs("maps")

    for i in range(0, len(maps)):
        print("地图名称：" + map_name_list[i])
        m = np.array(maps[i])
        with open("maps/{}.npy".format(map_name_list[i]), 'wb') as f:
            np.save(f, m)

        m = None
        with open("maps/{}.npy".format(map_name_list[i]), 'rb') as f:
            m = np.load(f)
        print(m)

    print("所有地图数据转换完毕")