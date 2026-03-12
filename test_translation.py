import numpy as np
from mapTools import (
    gen_base_map, random_obstacle_map, basic_region_partition, advanced_region_partition,
    map_to_binary, binary_to_map, submap_to_global_coords, global_to_submap_coords,
    visualize_multi_agent_path
)
import matplotlib.pyplot as plt

# 设置中文字体
plt.rcParams['font.family'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# 测试地图格式转换
def test_map_conversion():
    print("测试地图格式转换...")
    
    # 生成测试地图
    test_map = gen_base_map(10, 12, 2)
    print(f"原始地图形状: {test_map.shape}")
    
    # 转换为01地图
    binary_map = map_to_binary(test_map)
    print(f"01地图形状: {binary_map.shape}")
    
    # 转换回普通地图
    converted_map = binary_to_map(binary_map, [(0, 0), (9, 11)])
    print(f"转换回的地图形状: {converted_map.shape}")
    
    # 验证转换是否正确
    # 检查障碍物是否一致
    obstacle_match = np.array_equal((test_map == 1), (converted_map == 1))
    print(f"障碍物转换一致: {obstacle_match}")
    
    # 检查可通行区域是否一致
    free_match = np.array_equal((test_map == 0), (converted_map == 0) | (converted_map == 2))
    print(f"可通行区域转换一致: {free_match}")
    
    return obstacle_match and free_match

# 测试坐标转译
def test_coordinate_translation():
    print("\n测试坐标转译...")
    
    # 生成测试地图和区域划分
    test_map = gen_base_map(16, 19, 2)
    regions = basic_region_partition(test_map, 2)
    
    # 测试第一个区域的坐标转译
    region = regions[0]
    bounds = region['bounds']
    print(f"区域边界: {bounds}")
    
    # 测试子地图坐标转换为总图坐标
    submap_coords = (5, 5)  # 子地图中的坐标
    global_coords = submap_to_global_coords(submap_coords, bounds)
    print(f"子地图坐标 {submap_coords} 转换为总图坐标: {global_coords}")
    
    # 测试总图坐标转换为子地图坐标
    converted_submap_coords = global_to_submap_coords(global_coords, bounds)
    print(f"总图坐标 {global_coords} 转换回子地图坐标: {converted_submap_coords}")
    
    # 验证转换是否正确
    conversion_correct = (submap_coords == converted_submap_coords)
    print(f"坐标转换正确: {conversion_correct}")
    
    return conversion_correct

# 测试多机路径可视化
def test_multi_agent_visualization():
    print("\n测试多机路径可视化...")
    
    # 生成测试地图和区域划分
    test_map = gen_base_map(16, 19, 2)
    regions = advanced_region_partition(test_map, 3)
    
    # 为每个区域生成模拟路径
    paths = []
    for region in regions:
        # 生成模拟路径（随机点）
        import random
        submap = region['map']
        submap_rows, submap_cols = submap.shape
        path = []
        for i in range(10):
            row = random.randint(0, submap_rows-1)
            col = random.randint(0, submap_cols-1)
            path.append([row, col])
        paths.append(path)
    
    # 可视化多机路径
    visualize_multi_agent_path(test_map, regions, paths, "多机覆盖路径测试")
    print("多机路径可视化完成")
    return True

if __name__ == "__main__":
    print("开始测试转译层功能...")
    
    # 测试地图格式转换
    map_conversion_result = test_map_conversion()
    print(f"地图格式转换测试: {'通过' if map_conversion_result else '失败'}")
    
    # 测试坐标转译
    coordinate_translation_result = test_coordinate_translation()
    print(f"坐标转译测试: {'通过' if coordinate_translation_result else '失败'}")
    
    # 测试多机路径可视化
    visualization_result = test_multi_agent_visualization()
    print(f"多机路径可视化测试: {'通过' if visualization_result else '失败'}")
    
    # 总结测试结果
    all_tests_passed = map_conversion_result and coordinate_translation_result and visualization_result
    print(f"\n所有测试: {'全部通过' if all_tests_passed else '部分失败'}")
