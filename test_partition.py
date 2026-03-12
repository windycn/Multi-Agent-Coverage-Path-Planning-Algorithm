import numpy as np
from mapTools import gen_base_map, random_obstacle_map, basic_region_partition, advanced_region_partition
import matplotlib.pyplot as plt

# 设置中文字体
plt.rcParams['font.family'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# 测试初阶区域划分算法
def test_basic_partition():
    print("测试初阶区域划分算法...")
    
    # 生成测试地图
    test_maps = [
        ("基础地图", gen_base_map(16, 19, 2)),
        ("随机障碍地图", random_obstacle_map(10, 12))
    ]
    
    # 测试不同区域数量
    partition_counts = [2, 3, 4]
    
    for map_name, test_map in test_maps:
        print(f"\n测试地图: {map_name}")
        print(f"地图尺寸: {test_map.shape if isinstance(test_map, np.ndarray) else (len(test_map), len(test_map[0]))}")
        
        for num_regions in partition_counts:
            print(f"\n划分区域数量: {num_regions}")
            regions = basic_region_partition(test_map, num_regions)
            
            # 打印划分结果
            total_area = sum(region['area'] for region in regions)
            print(f"总可覆盖面积: {total_area}")
            print("各区域信息:")
            for i, region in enumerate(regions):
                start_row, start_col, end_row, end_col = region['bounds']
                area = region['area']
                area_ratio = (area / total_area) * 100 if total_area > 0 else 0
                print(f"区域 {i+1}: 边界=({start_row},{start_col})-({end_row},{end_col}), 面积={area}, 占比={area_ratio:.1f}%")
            
            # 可视化划分结果
            visualize_partition(test_map, regions, f"{map_name}_{num_regions}区域")

def visualize_partition(original_map, regions, title):
    """可视化区域划分结果"""
    # 转换为numpy数组便于处理
    if not isinstance(original_map, np.ndarray):
        original_map = np.array(original_map)
    
    # 创建可视化地图
    visual_map = np.copy(original_map)
    
    # 为每个区域分配不同的颜色
    colors = [3, 4, 5, 6]  # 不同区域的标记值
    
    for i, region in enumerate(regions):
        start_row, start_col, end_row, end_col = region['bounds']
        color = colors[i % len(colors)]
        
        # 标记区域边界
        for r in range(start_row, end_row + 1):
            if start_col < visual_map.shape[1]:
                visual_map[r][start_col] = color
            if end_col < visual_map.shape[1]:
                visual_map[r][end_col] = color
        
        for c in range(start_col, end_col + 1):
            if start_row < visual_map.shape[0]:
                visual_map[start_row][c] = color
            if end_row < visual_map.shape[0]:
                visual_map[end_row][c] = color
    
    # 绘制地图
    plt.figure(figsize=(10, 6))
    cmap = plt.cm.get_cmap('tab10', 7)  # 7种颜色
    bounds = [0, 1, 2, 3, 4, 5, 6, 7]
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)
    
    plt.imshow(visual_map, cmap=cmap, norm=norm)
    plt.colorbar(ticks=[0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5], 
                 label='0: 空白, 1: 障碍物, 2: 起始点, 3-6: 区域')
    plt.title(title)
    plt.grid(True, linewidth=0.5, alpha=0.5)
    plt.tight_layout()
    
    # 保存结果
    import os
    if not os.path.exists("output_images"):
        os.makedirs("output_images")
    plt.savefig(f"output_images/{title}.png")
    plt.show()

def test_advanced_partition():
    print("\n\n测试高阶区域划分算法...")
    
    # 生成测试地图
    test_maps = [
        ("基础地图", gen_base_map(16, 19, 2)),
        ("随机障碍地图", random_obstacle_map(10, 12))
    ]
    
    # 测试不同区域数量
    partition_counts = [2, 3, 4]
    
    for map_name, test_map in test_maps:
        print(f"\n测试地图: {map_name}")
        print(f"地图尺寸: {test_map.shape if isinstance(test_map, np.ndarray) else (len(test_map), len(test_map[0]))}")
        
        for num_regions in partition_counts:
            print(f"\n划分区域数量: {num_regions}")
            regions = advanced_region_partition(test_map, num_regions)
            
            # 打印划分结果
            total_area = sum(region['area'] for region in regions)
            total_steps = sum(region['estimated_steps'] for region in regions)
            print(f"总可覆盖面积: {total_area}")
            print(f"总估计步数: {total_steps}")
            print("各区域信息:")
            for i, region in enumerate(regions):
                start_row, start_col, end_row, end_col = region['bounds']
                area = region['area']
                steps = region['estimated_steps']
                area_ratio = (area / total_area) * 100 if total_area > 0 else 0
                steps_ratio = (steps / total_steps) * 100 if total_steps > 0 else 0
                print(f"区域 {i+1}: 边界=({start_row},{start_col})-({end_row},{end_col}), 面积={area}({area_ratio:.1f}%), 估计步数={steps}({steps_ratio:.1f}%)")
            
            # 可视化划分结果
            visualize_advanced_partition(test_map, regions, f"高级划分_{map_name}_{num_regions}区域")

def visualize_advanced_partition(original_map, regions, title):
    """可视化高阶区域划分结果"""
    # 转换为numpy数组便于处理
    if not isinstance(original_map, np.ndarray):
        original_map = np.array(original_map)
    
    # 创建可视化地图
    visual_map = np.copy(original_map)
    
    # 为每个区域分配不同的颜色
    colors = [3, 4, 5, 6]  # 不同区域的标记值
    
    for i, region in enumerate(regions):
        color = colors[i % len(colors)]
        # 标记区域内的所有细胞
        for cell in region['cells']:
            r, c = cell
            if r < visual_map.shape[0] and c < visual_map.shape[1]:
                visual_map[r][c] = color
        
        # 标记区域边界
        start_row, start_col, end_row, end_col = region['bounds']
        for r in range(start_row, end_row + 1):
            if start_col < visual_map.shape[1]:
                visual_map[r][start_col] = color + 10
            if end_col < visual_map.shape[1]:
                visual_map[r][end_col] = color + 10
        
        for c in range(start_col, end_col + 1):
            if start_row < visual_map.shape[0]:
                visual_map[start_row][c] = color + 10
            if end_row < visual_map.shape[0]:
                visual_map[end_row][c] = color + 10
    
    # 绘制地图
    plt.figure(figsize=(10, 6))
    cmap = plt.cm.get_cmap('tab20', 20)  # 20种颜色
    bounds = list(range(20))
    norm = plt.cm.colors.BoundaryNorm(bounds, cmap.N)
    
    plt.imshow(visual_map, cmap=cmap, norm=norm)
    plt.colorbar(ticks=[0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 6.5, 13.5, 14.5, 15.5, 16.5], 
                 label='0: 空白, 1: 障碍物, 2: 起始点, 3-6: 区域, 13-16: 区域边界')
    plt.title(title)
    plt.grid(True, linewidth=0.5, alpha=0.5)
    plt.tight_layout()
    
    # 保存结果
    import os
    if not os.path.exists("output_images"):
        os.makedirs("output_images")
    plt.savefig(f"output_images/{title}.png")
    plt.show()

if __name__ == "__main__":
    test_basic_partition()
    test_advanced_partition()
