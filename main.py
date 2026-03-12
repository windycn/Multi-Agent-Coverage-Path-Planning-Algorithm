from getPath import plan_coverage_path
from mapTools import (
    gen_base_map, randomStartPoint, random_obstacle_map, map2np,
    basic_region_partition, advanced_region_partition, visualize_multi_agent_path
)
import numpy as np

# 多机覆盖路径规划示例
def multi_agent_coverage_example():
    print("\n=== 多机覆盖路径规划示例 ===")
    
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
    
    # 打印各区域信息
    print("\n5. 各区域信息:")
    for i, region in enumerate(regions):
        start_row, start_col, end_row, end_col = region['bounds']
        area = region['area']
        print(f"   区域 {i+1}: 边界=({start_row},{start_col})-({end_row},{end_col}), 面积={area}")
    
    print("\n多机覆盖路径规划示例完成！")

if __name__ == "__main__":
    print("=== 多机覆盖路径规划算法 ===")
    print("1. 单机路径规划示例")
    print("2. 多机覆盖路径规划示例")
    choice = input("请选择 (1/2): ")
    
    if choice == '1':
        # 预设地图
        Test_map = [[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1],
                   [0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1],
                   [0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 0, 0],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1],
                   [0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1],
                   [0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1],
                   [0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                   [0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
                   [0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1],
                   [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2]]
        Cover1 = [
            [2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0],
            [1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0],
            [1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0],
            [1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0],
            [1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0],
        ]
        Cover2 = [
            [0, 0, 0, 0],
            [0, 1, 1, 1],
            [0, 1, 1, 1],
            [0, 0, 0, 0],
            [0, 1, 1, 0],
            [0, 1, 1, 0],
            [0, 0, 0, 0],
            [2, 1, 1, 0],
            [0, 1, 1, 0],
            [0, 0, 0, 0],
            [0, 1, 1, 0],
            [0, 1, 1, 0],
            [0, 1, 1, 0],
            [0, 1, 1, 0],
            [0, 1, 1, 0],
            [0, 0, 0, 0]
        ]
        Cover3 = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0],
            [1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0],
            [1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0],
            [1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0]
        ]
        maps = [Test_map, Cover1, Cover2, Cover3]
        map_name_list = ["Test_map", "Cover1", "Cover2", "Cover3"]

        # 生成基础地图
        Base_map = gen_base_map()
        # 生成随机地图
        Random_obstacle_map = random_obstacle_map()
        # 随机添加一个起始点
        Base_map = randomStartPoint(Base_map, 1)
        Random_obstacle_map = randomStartPoint(Random_obstacle_map, 1)

        maps.append(Base_map)
        map_name_list.append("Base_map")

        maps.append(Random_obstacle_map)
        map_name_list.append("Random_obstacle_map")

        # 将所有数组地图转换成npy地图，储存在/map/XX.npy
        map2np(maps, map_name_list)

        # 载入地图，生成规划
        best_trajectory_list = plan_coverage_path(map_name_list, True, True, False)
        print(best_trajectory_list)
    
    elif choice == '2':
        # 运行多机覆盖路径规划示例
        multi_agent_coverage_example()
    
    else:
        print("无效选择！")