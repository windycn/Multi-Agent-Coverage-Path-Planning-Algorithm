## 多机覆盖路径规划算法 Multi-Agent Coverage Path Planning Algorithm

### 整体思路 Overall  
![整体流程框架图](https://github.com/windycn/Multi-Agent-Coverage-Path-Planning-Algorithm/blob/main/images/framework.png)

将多机问题转化为单机问题，每个个体（无人机/UAV）覆盖式扫描每个划分好的地图，最后合并每个地图下的路径规划，完成多机协同覆盖搜索。

每个划分地图都包含一个个体在内，因此有多种划分方式，这里的区域划分算法效果可能有初阶、中阶和高阶之分。理想情况下，覆盖所有的地图划分需要尽可能地均匀分配地图，以确保总时长最少。更高阶的区域划分需要考虑到空间的连通性和每个区间的步数，划分出更加合理的区间。

Transforming the multi-agent problem into a single-agent problem, where each agent (UAV) conducts coverage scanning over individually partitioned maps. Finally, the path planning for each map is merged to achieve collaborative multi-agent coverage search.

Each partitioned map includes one agent, giving rise to multiple partitioning approaches. The quality of the area partitioning algorithm can be categorized as basic, intermediate, or advanced. Ideally, achieving an even distribution of maps for coverage is essential to minimize the overall time. Advanced area partitioning methods consider spatial connectivity and the number of steps in each partition to create more rational divisions.

单机路径规划算法（启发式）Single-Agent Path Planning Algorithm (Heuristic):

![算法原理](https://github.com/windycn/Multi-Agent-Coverage-Path-Planning-Algorithm/blob/main/images/alg.png)

### 待加算法 Todo List

- [x] 单机路径规划算法（启发式）Single-Agent Path Planning Algorithm (Heuristic)

- [ ] 初阶区域划分算法 Basic Region Partitioning Algorithm

- [ ] 高阶区域划分算法 Advanced Region Partitioning Algorithm
      
- [ ] 转译层（①普通地图与01地图互转；②划分后01地图对应至总图位置坐标转译）Translation layer ( ①Common map and 01 map intertransfer; ② After division 01 map corresponds to the general map position coordinate translation )

### 安装和使用 Installation & Usage
```bash
conda create -n CoPath python=3.8
conda activate CoPath
pip install -r requirements.txt
```

```bash
python main.py
```

### 现有函数说明 Existing function specification
- PathPlanningCore 算法核心层
- getPath 算法解算层
```python
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
```
- mapTools 地图工具
```python
def gen_base_map(rows=16, cols=19, obstacle_size=2):
    '''
    生成基础的数组地图，可以手动调整障碍后调用map2np生成持久储存使用的npy地图

    :param rows: 行数 (空格为0)
    :param cols: 列数 (空格为0)
    :param obstacle_size: 障碍尺寸 (障碍默认2X2大小，为1)
    :return: grid: 生成的map数组
    '''

def random_obstacle_map(rows=10, cols=12):
    '''
    生成随机障碍地图

    :param rows: 行数 (空格为0)
    :param cols: 列数 (空格为0)
    :return: grid: 生成的map数组
    '''

def randomStartPoint(input_map: list, startpoint=1):
    '''
    随机生成起始点 (置为2) 加入到地图中(随机四周放点，四周所有的[0][0]和首行[0]与尾行[-1])

    :param input_map: 输入的地图
    :param startpoint: 起始点数量，默认为1（一次规划只能规划最开始的起始点），划分后可以进行多个
    :return: new_map: 含随机起始点的新地图
    '''

def map2np(maps: list, map_name_list: list):
    '''
    将数组地图转换成持久储存使用的npy地图

    :param maps: 地图数据列表 (嵌套数组)
    :param map_name_list: 地图名字列表 (也是保存的文件名maps/map_name.npy)
    :return: None
    '''
```

### 效果展示 Results
基础地图[Base_map]  
![Base_map](https://github.com/windycn/Multi-Agent-Coverage-Path-Planning-Algorithm/blob/main/images/Base_map.png)

修改后基础地图[Test_map]  
![Test_map](https://github.com/windycn/Multi-Agent-Coverage-Path-Planning-Algorithm/blob/main/images/Test_map.png)

随机障碍地图[Random_obstacle_map]  
![Random_obstacle_map](https://github.com/windycn/Multi-Agent-Coverage-Path-Planning-Algorithm/blob/main/images/Random_obstacle_map.png)

划分区域1[Cover1]  
![Cover1](https://github.com/windycn/Multi-Agent-Coverage-Path-Planning-Algorithm/blob/main/images/Cover1.png)

划分区域2[Cover2]  
![Cover2](https://github.com/windycn/Multi-Agent-Coverage-Path-Planning-Algorithm/blob/main/images/Cover2.png)

划分区域3[Cover3]  
![Cover3](https://github.com/windycn/Multi-Agent-Coverage-Path-Planning-Algorithm/blob/main/images/Cover3.png)
