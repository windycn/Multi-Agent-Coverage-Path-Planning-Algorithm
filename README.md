## 多机覆盖路径规划算法 Multi-Agent Coverage Path Planning Algorithm

### 整体思路 Overall  
![整体流程框架图](https://github.com/windycn/Multi-Agent-Coverage-Path-Planning-Algorithm/blob/main/images/framework.png)

将多机问题转化为单机问题，每个个体（无人机/UAV）覆盖式扫描每个划分好的地图，最后合并每个地图下的路径规划，完成多机协同覆盖搜索。

每个划分地图都包含一个个体在内，因此有多种划分方式，这里的区域划分算法效果可能有初阶、中阶和高阶之分。理想情况下，覆盖所有的地图划分需要尽可能地均匀分配地图，以确保总时长最少。更高阶的区域划分需要考虑到空间的连通性和每个区间的步数，划分出更加合理的区间。

Transforming the multi-agent problem into a single-agent problem, where each agent (UAV) conducts coverage scanning over individually partitioned maps. Finally, the path planning for each map is merged to achieve collaborative multi-agent coverage search.

Each partitioned map includes one agent, giving rise to multiple partitioning approaches. The quality of the area partitioning algorithm can be categorized as basic, intermediate, or advanced. Ideally, achieving an even distribution of maps for coverage is essential to minimize the overall time. Advanced area partitioning methods consider spatial connectivity and the number of steps in each partition to create more rational divisions.

### 待加算法 Todo List

- [x] 单机路径规划算法（启发式）Single-Agent Path Planning Algorithm (Heuristic)

- [ ] 初阶区域划分算法 Basic Region Partitioning Algorithm

- [ ] 高阶区域划分算法 Advanced Region Partitioning Algorithm

### 安装和使用 Installation & Usage
```bash
conda create -n CoPath python=3.8
conda activate CoPath
pip install -r requirements.txt
```

```bash
python main.py
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

### 引用和参考 References & Citations
[GA-TSPCPP](https://github.com/WJTung/GA-TSPCPP)  
[coverage-path-planning](https://github.com/rodriguesrenato/coverage-path-planning)
