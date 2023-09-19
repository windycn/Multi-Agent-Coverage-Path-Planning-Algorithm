from getPath import plan_coverage_path
from mapTools import gen_base_map, randomStartPoint, random_obstacle_map, map2np

if __name__ == "__main__":
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