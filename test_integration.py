import numpy as np
from mapTools import (
    gen_base_map, random_obstacle_map, basic_region_partition, advanced_region_partition,
    map_to_binary, binary_to_map, submap_to_global_coords, visualize_multi_agent_path
)
from getPath import plan_coverage_path
import matplotlib.pyplot as plt
import time

# 设置中文字体
plt.rcParams['font.family'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# 集成测试：完整的多机覆盖路径规划流程
def test_multi_agent_coverage():
    print("开始集成测试：多机覆盖路径规划")
    
    # 生成测试地图
    test_map = gen_base_map(16, 19, 2)
    print(f"测试地图尺寸: {test_map.shape}")
    
    # 选择区域划分算法
    print("\n1. 选择区域划分算法：")
    print("   1. 初阶区域划分算法")
    print("   2. 高阶区域划分算法")
    algorithm_choice = input("   请选择 (1/2): ")
    
    # 输入智能体数量
    num_agents = int(input("\n2. 请输入智能体数量 (2-4): "))
    
    # 记录开始时间
    start_time = time.time()
    
    # 执行区域划分
    if algorithm_choice == '1':
        print("\n使用初阶区域划分算法...")
        regions = basic_region_partition(test_map, num_agents)
    else:
        print("\n使用高阶区域划分算法...")
        regions = advanced_region_partition(test_map, num_agents)
    
    # 保存子地图
    submap_names = []
    for i, region in enumerate(regions):
        submap = region['map']
        submap_name = f"submap_{i+1}"
        submap_names.append(submap_name)
        
        # 保存为npy文件
        import os
        if not os.path.exists("maps"):
            os.makedirs("maps")
        np.save(f"maps/{submap_name}.npy", submap)
    
    # 为每个子地图规划路径
    print("\n3. 为每个子地图规划路径...")
    paths = plan_coverage_path(submap_names, isprint=False, isconsole=False)
    
    # 提取路径点
    path_points = []
    for path in paths:
        points = path['Path_point_list']
        path_points.append(points)
    
    # 可视化多机路径
    print("\n4. 可视化多机覆盖路径...")
    visualize_multi_agent_path(test_map, regions, path_points, f"多机覆盖路径规划_{num_agents}智能体")
    
    # 计算执行时间
    execution_time = time.time() - start_time
    print(f"\n5. 执行完成！")
    print(f"   执行时间: {execution_time:.2f} 秒")
    print(f"   智能体数量: {num_agents}")
    print(f"   区域数量: {len(regions)}")
    
    # 打印各区域信息
    print("\n6. 各区域信息:")
    for i, region in enumerate(regions):
        start_row, start_col, end_row, end_col = region['bounds']
        area = region['area']
        print(f"   区域 {i+1}: 边界=({start_row},{start_col})-({end_row},{end_col}), 面积={area}")
    
    return True

# 性能测试
def test_performance():
    print("\n\n开始性能测试...")
    
    # 测试不同大小的地图
    map_sizes = [(10, 12), (16, 19), (20, 25)]
    agent_counts = [2, 3, 4]
    
    for rows, cols in map_sizes:
        print(f"\n测试地图尺寸: {rows}x{cols}")
        test_map = gen_base_map(rows, cols, 2)
        
        for num_agents in agent_counts:
            start_time = time.time()
            
            # 使用高阶区域划分算法
            regions = advanced_region_partition(test_map, num_agents)
            
            # 保存子地图
            submap_names = []
            for i, region in enumerate(regions):
                submap = region['map']
                submap_name = f"temp_submap_{i+1}"
                submap_names.append(submap_name)
                np.save(f"maps/{submap_name}.npy", submap)
            
            # 规划路径
            paths = plan_coverage_path(submap_names, isprint=False, isconsole=False)
            
            execution_time = time.time() - start_time
            print(f"   {num_agents}智能体: {execution_time:.2f} 秒")
    
    # 清理临时文件
    import os
    for file in os.listdir("maps"):
        if file.startswith("temp_submap_"):
            os.remove(os.path.join("maps", file))
    
    print("\n性能测试完成！")

if __name__ == "__main__":
    # 运行集成测试
    test_multi_agent_coverage()
    
    # 运行性能测试
    test_performance()
    
    print("\n所有集成测试完成！")
