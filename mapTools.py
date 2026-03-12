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


def basic_region_partition(input_map: list, num_regions: int):
    '''
    初阶区域划分算法，将地图均匀划分为指定数量的子区域

    :param input_map: 输入的地图
    :param num_regions: 要划分的区域数量
    :return: regions: 划分后的子区域列表，每个子区域包含地图数据和边界信息
    '''
    # 使用深拷贝创建副本
    input_map = copy.deepcopy(input_map)
    rows = len(input_map)
    cols = len(input_map[0])
    
    regions = []
    
    if num_regions == 1:
        # 不需要划分，返回整个地图
        regions.append({
            'map': input_map,
            'bounds': (0, 0, rows-1, cols-1),  # (start_row, start_col, end_row, end_col)
            'area': rows * cols
        })
        return regions
    
    # 根据地图形状决定划分方式
    if rows >= cols:
        # 按行划分
        rows_per_region = rows // num_regions
        remainder = rows % num_regions
        
        start_row = 0
        for i in range(num_regions):
            # 分配剩余行
            current_rows = rows_per_region + (1 if i < remainder else 0)
            end_row = start_row + current_rows - 1
            
            # 提取子区域
            sub_map = input_map[start_row:end_row+1, :] if isinstance(input_map, np.ndarray) else [row[:] for row in input_map[start_row:end_row+1]]
            
            # 计算子区域面积（排除障碍物）
            area = 0
            for r in sub_map:
                for c in r:
                    if c == 0:
                        area += 1
            
            regions.append({
                'map': sub_map,
                'bounds': (start_row, 0, end_row, cols-1),
                'area': area
            })
            
            start_row = end_row + 1
    else:
        # 按列划分
        cols_per_region = cols // num_regions
        remainder = cols % num_regions
        
        start_col = 0
        for i in range(num_regions):
            # 分配剩余列
            current_cols = cols_per_region + (1 if i < remainder else 0)
            end_col = start_col + current_cols - 1
            
            # 提取子区域
            if isinstance(input_map, np.ndarray):
                sub_map = input_map[:, start_col:end_col+1]
            else:
                sub_map = []
                for row in input_map:
                    sub_map.append(row[start_col:end_col+1])
            
            # 计算子区域面积（排除障碍物）
            area = 0
            for r in sub_map:
                for c in r:
                    if c == 0:
                        area += 1
            
            regions.append({
                'map': sub_map,
                'bounds': (0, start_col, rows-1, end_col),
                'area': area
            })
            
            start_col = end_col + 1
    
    # 为每个子区域添加起始点
    for i, region in enumerate(regions):
        # 查找子区域内的可行起始点（边界位置）
        sub_map = region['map']
        start_row, start_col, end_row, end_col = region['bounds']
        
        # 尝试在子区域的边界找到可行的起始点
        possible_starts = []
        
        # 上边界
        for c in range(start_col, end_col + 1):
            if input_map[start_row][c] == 0:
                possible_starts.append((start_row, c))
        
        # 下边界
        for c in range(start_col, end_col + 1):
            if input_map[end_row][c] == 0:
                possible_starts.append((end_row, c))
        
        # 左边界
        for r in range(start_row, end_row + 1):
            if input_map[r][start_col] == 0:
                possible_starts.append((r, start_col))
        
        # 右边界
        for r in range(start_row, end_row + 1):
            if input_map[r][end_col] == 0:
                possible_starts.append((r, end_col))
        
        # 如果找到可行的起始点，选择第一个作为该区域的起始点
        if possible_starts:
            start_r, start_c = possible_starts[0]
            # 在原始地图和子地图中标记起始点
            input_map[start_r][start_c] = 2
            # 在子地图中计算相对坐标
            sub_r = start_r - region['bounds'][0]
            sub_c = start_c - region['bounds'][1]
            if isinstance(sub_map, np.ndarray):
                sub_map[sub_r][sub_c] = 2
            else:
                sub_map[sub_r][sub_c] = 2
    
    return regions


def advanced_region_partition(input_map: list, num_regions: int):
    '''
    高阶区域划分算法，考虑空间连通性和负载均衡

    :param input_map: 输入的地图
    :param num_regions: 要划分的区域数量
    :return: regions: 划分后的子区域列表，每个子区域包含地图数据和边界信息
    '''
    # 使用深拷贝创建副本
    input_map = copy.deepcopy(input_map)
    rows = len(input_map)
    cols = len(input_map[0])
    
    if not isinstance(input_map, np.ndarray):
        input_map = np.array(input_map)
    
    # 步骤1：识别所有连通区域
    connected_regions = identify_connected_regions(input_map)
    print(f"识别到 {len(connected_regions)} 个连通区域")
    
    # 步骤2：如果连通区域数量少于智能体数量，将大的连通区域分割
    if len(connected_regions) < num_regions:
        split_regions = []
        for region in connected_regions:
            # 如果区域足够大，分割它
            if len(region['cells']) > num_regions:
                # 基于网格划分分割区域
                sub_regions = split_large_region(region, max(1, num_regions // len(connected_regions)))
                split_regions.extend(sub_regions)
            else:
                split_regions.append(region)
        connected_regions = split_regions
    
    # 步骤3：估计每个连通区域的覆盖步数
    for region in connected_regions:
        region['estimated_steps'] = estimate_coverage_steps(region['cells'])
    
    # 步骤4：基于贪心算法分配连通区域
    agent_regions = assign_regions(connected_regions, num_regions)
    
    # 步骤5：构建最终的子区域
    final_regions = []
    for i, agent_region in enumerate(agent_regions):
        # 跳过空区域
        if not agent_region['cells']:
            continue
        
        # 计算边界
        min_row = min(cell[0] for cell in agent_region['cells'])
        max_row = max(cell[0] for cell in agent_region['cells'])
        min_col = min(cell[1] for cell in agent_region['cells'])
        max_col = max(cell[1] for cell in agent_region['cells'])
        
        # 提取子地图
        sub_map = np.copy(input_map[min_row:max_row+1, min_col:max_col+1])
        
        # 计算面积
        area = len(agent_region['cells'])
        
        # 为子区域添加起始点
        start_point = find_start_point(agent_region['cells'])
        if start_point:
            # 在原始地图中标记起始点
            input_map[start_point[0]][start_point[1]] = 2
            # 在子地图中标记起始点
            sub_row = start_point[0] - min_row
            sub_col = start_point[1] - min_col
            sub_map[sub_row][sub_col] = 2
        
        final_regions.append({
            'map': sub_map,
            'bounds': (min_row, min_col, max_row, max_col),
            'area': area,
            'estimated_steps': agent_region['estimated_steps'],
            'cells': agent_region['cells']
        })
    
    # 确保至少返回一个区域
    if not final_regions and num_regions > 0:
        # 返回整个地图作为一个区域
        final_regions.append({
            'map': input_map,
            'bounds': (0, 0, rows-1, cols-1),
            'area': np.sum(input_map == 0),
            'estimated_steps': estimate_coverage_steps([(i, j) for i in range(rows) for j in range(cols) if input_map[i][j] == 0]),
            'cells': [(i, j) for i in range(rows) for j in range(cols) if input_map[i][j] == 0]
        })
    
    return final_regions

def split_large_region(region, num_subregions):
    '''
    分割大的连通区域为多个子区域

    :param region: 要分割的连通区域
    :param num_subregions: 子区域数量
    :return: 分割后的子区域列表
    '''
    cells = region['cells']
    
    # 计算区域的边界
    min_row = min(cell[0] for cell in cells)
    max_row = max(cell[0] for cell in cells)
    min_col = min(cell[1] for cell in cells)
    max_col = max(cell[1] for cell in cells)
    
    # 计算区域的宽度和高度
    width = max_col - min_col + 1
    height = max_row - min_row + 1
    
    # 基于宽度和高度决定分割方向
    if width > height:
        # 按列分割
        cols_per_region = width // num_subregions
        remainder = width % num_subregions
        
        sub_regions = []
        start_col = min_col
        for i in range(num_subregions):
            current_cols = cols_per_region + (1 if i < remainder else 0)
            end_col = start_col + current_cols - 1
            
            # 收集该列范围内的细胞
            sub_cells = [(r, c) for r, c in cells if start_col <= c <= end_col]
            if sub_cells:
                sub_regions.append({
                    'cells': sub_cells,
                    'area': len(sub_cells)
                })
            
            start_col = end_col + 1
    else:
        # 按行分割
        rows_per_region = height // num_subregions
        remainder = height % num_subregions
        
        sub_regions = []
        start_row = min_row
        for i in range(num_subregions):
            current_rows = rows_per_region + (1 if i < remainder else 0)
            end_row = start_row + current_rows - 1
            
            # 收集该行范围内的细胞
            sub_cells = [(r, c) for r, c in cells if start_row <= r <= end_row]
            if sub_cells:
                sub_regions.append({
                    'cells': sub_cells,
                    'area': len(sub_cells)
                })
            
            start_row = end_row + 1
    
    return sub_regions

def identify_connected_regions(map_array):
    '''
    识别地图中的连通区域

    :param map_array: 地图数组
    :return: 连通区域列表，每个区域包含细胞列表和面积
    '''
    rows, cols = map_array.shape
    visited = np.zeros_like(map_array, dtype=bool)
    connected_regions = []
    
    # 定义四个方向
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    for i in range(rows):
        for j in range(cols):
            # 如果当前位置是空白且未访问
            if map_array[i][j] == 0 and not visited[i][j]:
                # 使用BFS识别连通区域
                queue = [(i, j)]
                visited[i][j] = True
                cells = [(i, j)]
                
                while queue:
                    x, y = queue.pop(0)
                    for dx, dy in directions:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < rows and 0 <= ny < cols:
                            if map_array[nx][ny] == 0 and not visited[nx][ny]:
                                visited[nx][ny] = True
                                queue.append((nx, ny))
                                cells.append((nx, ny))
                
                # 只添加有一定大小的区域
                if len(cells) > 1:
                    connected_regions.append({
                        'cells': cells,
                        'area': len(cells)
                    })
    
    return connected_regions

def estimate_coverage_steps(cells):
    '''
    估计覆盖区域所需的步数

    :param cells: 区域中的细胞列表
    :return: 估计的步数
    '''
    # 基于区域大小和形状估计步数
    # 简单估计：区域面积的平方根乘以一个系数
    area = len(cells)
    if area == 0:
        return 0
    
    # 计算区域的宽度和高度
    min_row = min(cell[0] for cell in cells)
    max_row = max(cell[0] for cell in cells)
    min_col = min(cell[1] for cell in cells)
    max_col = max(cell[1] for cell in cells)
    
    width = max_col - min_col + 1
    height = max_row - min_row + 1
    
    # 估计步数：考虑区域的周长和面积
    perimeter = 2 * (width + height)
    estimated_steps = int(area * 1.2 + perimeter * 0.5)
    
    return estimated_steps

def assign_regions(connected_regions, num_regions):
    '''
    基于贪心算法分配连通区域给不同的智能体

    :param connected_regions: 连通区域列表
    :param num_regions: 智能体数量
    :return: 分配给每个智能体的区域
    '''
    # 初始化智能体区域
    agent_regions = []
    for i in range(num_regions):
        agent_regions.append({
            'cells': [],
            'estimated_steps': 0
        })
    
    # 按面积降序排序连通区域
    sorted_regions = sorted(connected_regions, key=lambda x: x['area'], reverse=True)
    
    # 贪心分配：将最大的区域分配给当前负载最小的智能体
    for region in sorted_regions:
        # 找到当前负载最小的智能体
        min_agent = min(agent_regions, key=lambda x: x['estimated_steps'])
        # 分配区域
        min_agent['cells'].extend(region['cells'])
        min_agent['estimated_steps'] += region['estimated_steps']
    
    return agent_regions

def find_start_point(cells):
    '''
    在区域中找到合适的起始点

    :param cells: 区域中的细胞列表
    :return: 起始点坐标
    '''
    if not cells:
        return None
    
    # 优先选择边界位置作为起始点
    # 计算区域的边界
    min_row = min(cell[0] for cell in cells)
    max_row = max(cell[0] for cell in cells)
    min_col = min(cell[1] for cell in cells)
    max_col = max(cell[1] for cell in cells)
    
    # 检查四个角落
    corners = [(min_row, min_col), (min_row, max_col), (max_row, min_col), (max_row, max_col)]
    for corner in corners:
        if corner in cells:
            return corner
    
    # 如果没有角落，选择上边界的点
    for col in range(min_col, max_col + 1):
        if (min_row, col) in cells:
            return (min_row, col)
    
    # 选择第一个点
    return cells[0]


def map_to_binary(map_array):
    '''
    将普通地图转换为01地图
    0: 可通行区域
    1: 障碍物

    :param map_array: 普通地图数组
    :return: 01地图数组
    '''
    if not isinstance(map_array, np.ndarray):
        map_array = np.array(map_array)
    
    # 创建01地图
    binary_map = np.zeros_like(map_array, dtype=int)
    binary_map[map_array == 1] = 1  # 障碍物为1
    
    return binary_map


def binary_to_map(binary_map, start_positions=None):
    '''
    将01地图转换为普通地图
    0: 可通行区域
    1: 障碍物
    2: 起始点

    :param binary_map: 01地图数组
    :param start_positions: 起始点位置列表，默认为None
    :return: 普通地图数组
    '''
    if not isinstance(binary_map, np.ndarray):
        binary_map = np.array(binary_map)
    
    # 创建普通地图
    map_array = np.zeros_like(binary_map, dtype=int)
    map_array[binary_map == 1] = 1  # 障碍物为1
    
    # 添加起始点
    if start_positions:
        for pos in start_positions:
            if 0 <= pos[0] < map_array.shape[0] and 0 <= pos[1] < map_array.shape[1]:
                map_array[pos[0]][pos[1]] = 2
    
    return map_array


def submap_to_global_coords(submap_coords, region_bounds):
    '''
    将子地图坐标转换为总图坐标

    :param submap_coords: 子地图中的坐标 (row, col)
    :param region_bounds: 区域边界 (start_row, start_col, end_row, end_col)
    :return: 总图中的坐标 (row, col)
    '''
    start_row, start_col, _, _ = region_bounds
    global_row = start_row + submap_coords[0]
    global_col = start_col + submap_coords[1]
    return (global_row, global_col)


def global_to_submap_coords(global_coords, region_bounds):
    '''
    将总图坐标转换为子地图坐标

    :param global_coords: 总图中的坐标 (row, col)
    :param region_bounds: 区域边界 (start_row, start_col, end_row, end_col)
    :return: 子地图中的坐标 (row, col)
    '''
    start_row, start_col, _, _ = region_bounds
    submap_row = global_coords[0] - start_row
    submap_col = global_coords[1] - start_col
    return (submap_row, submap_col)


def visualize_multi_agent_path(original_map, regions, paths, title="多机覆盖路径"):
    '''
    在总图上可视化多机协同覆盖路径

    :param original_map: 原始地图
    :param regions: 划分后的区域列表
    :param paths: 各区域的路径列表
    :param title: 标题
    '''
    import matplotlib.pyplot as plt
    
    # 转换为numpy数组便于处理
    if not isinstance(original_map, np.ndarray):
        original_map = np.array(original_map)
    
    # 创建可视化地图
    visual_map = np.copy(original_map)
    
    # 为每个区域分配不同的颜色
    colors = ['r', 'g', 'b', 'y', 'm', 'c']
    
    # 绘制区域边界
    for i, region in enumerate(regions):
        start_row, start_col, end_row, end_col = region['bounds']
        color = colors[i % len(colors)]
        
        # 绘制区域边界
        plt.plot([start_col, end_col, end_col, start_col, start_col], 
                 [start_row, start_row, end_row, end_row, start_row], 
                 color=color, linestyle='--', linewidth=1)
    
    # 绘制路径
    for i, (region, path) in enumerate(zip(regions, paths)):
        color = colors[i % len(colors)]
        
        # 提取路径点
        path_points = []
        for point in path:
            if isinstance(point, list) and len(point) >= 2:
                # 转换为总图坐标
                global_point = submap_to_global_coords((point[0], point[1]), region['bounds'])
                path_points.append(global_point)
        
        # 绘制路径
        if path_points:
            x = [p[1] for p in path_points]
            y = [p[0] for p in path_points]
            plt.plot(x, y, color=color, marker='o', markersize=3, linewidth=1, label=f'智能体 {i+1}')
    
    # 绘制地图
    plt.imshow(visual_map, cmap='Greys', alpha=0.5)
    
    # 添加图例
    plt.legend()
    plt.title(title)
    plt.grid(True, linewidth=0.5, alpha=0.5)
    plt.tight_layout()
    
    # 保存结果
    import os
    if not os.path.exists("output_images"):
        os.makedirs("output_images")
    plt.savefig(f"output_images/{title}.png")
    plt.show()