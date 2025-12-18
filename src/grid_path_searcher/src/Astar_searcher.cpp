#include "Astar_searcher.hpp"

using namespace std;
using namespace Eigen;

// 初始化栅格地图
void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, 
                                   int max_x_id, int max_y_id, int max_z_id)
{   
    // 设置地图的下界坐标
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    // 设置地图的上界坐标
    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    // 设置栅格的尺寸
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    // 设置栅格分辨率和其倒数
    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    // 分配占用栅格数据数组
    data = new uint8_t[GLXYZ_SIZE];
    // 初始化栅格数据为未占用状态
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    // 创建三维网格节点数组
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            // 初始化每个网格节点
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

// 重置单个栅格节点
void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;  // 节点状态：0-未访问，1-开集，-1-闭集
    ptr->cameFrom = NULL;  // 清除父节点指针
    ptr->gScore = inf;  // 从起点到该节点的代价
    ptr->fScore = inf;  // 总代价 f(n) = g(n) + h(n)
    ptr->dir = Eigen::Vector3i::Zero();  // 重置方向为零向量
}

// 重置所有已使用过的栅格
void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

// 设置障碍物
void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{
    // 检查坐标是否在地图范围内，否则直接返回
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    // 将连续坐标转换为栅格索引
    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      

    // 标记该栅格为障碍物（占用）三维转一维占用
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

// 获取所有已访问过的节点
vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // 可视化开集和闭集中的所有节点
                if(GridNodeMap[i][j][k]->id == -1)  // 仅可视化闭集中的节点
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    RCLCPP_WARN(rclcpp::get_logger("astar_searcher"), "visited_nodes size : %zu", visited_nodes.size());
    return visited_nodes;
}


// 将栅格索引转换为世界坐标（栅格中心点）
Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    // 加0.5是为了获取栅格中心点坐标
    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

// 将世界坐标转换为栅格索引
Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    // 坐标转换索引，并限制在有效范围内
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
    return idx;
}

// 坐标舍入到栅格中心
Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

// 判断栅格索引对应的栅格是否被占用
inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

// 判断栅格索引对应的栅格是否空闲
inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

// 判断指定坐标的栅格是否被占用
inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

// 判断指定坐标的栅格是否空闲
inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

// 获取当前节点的所有后继节点及其边代价
inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, 
                                          vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
            for (int dz = -1; dz <= 1; dz++)
            {
                // 获取邻居索引
                int i = currentPtr->index(0) + dx;
                int j = currentPtr->index(1) + dy;
                int k = currentPtr->index(2) + dz;
                // 跳过在开集或闭集中的节点
                // if(GridNodeMap[i][j][k]->id != 0)
                //     continue;
                // 跳过当前节点自身
                if (dx == 0 && dy == 0 && dz == 0)
                    continue;
                // 检查邻居是否在地图范围内且为空闲
                if (isFree(i, j, k))
                { 
                    double cost = sqrt(dx * dx + dy * dy + dz * dz);
                    neighborPtrSets.push_back(GridNodeMap[i][j][k]);
                    edgeCostSets.push_back(cost);
                }
                    
            }
    /*
    *
    步骤 4: 完成 AstarPathFinder::AstarGetSucc 函数
    请在下方编写代码：获取当前节点的邻居节点及其代价
    
    提示：
    1. 遍历当前节点的26个邻居（3D情况）或8个邻居（2D情况）
    2. 检查邻居是否在地图范围内
    3. 检查邻居是否被占用
    4. 计算从当前节点到邻居的代价
    5. 将有效邻居加入neighborPtrSets，代价加入edgeCostSets
    *
    */
}

// 启发式函数（评估函数）
double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    // double Manhattan = abs(node1->index(0) - node2->index(0)) + 
    //             abs(node1->index(1) - node2->index(1)) + 
    //             abs(node1->index(2) - node2->index(2));
    // double Euclidean = sqrt( pow((node1->index(0) - node2->index(0)),2) + 
    //             pow((node1->index(1) - node2->index(1)),2) + 
    //             pow((node1->index(2) - node2->index(2)),2));
    
    // double Diagonal = max( max( abs(node1->index(0) - node2->index(0)), 
    //                             abs(node1->index(1) - node2->index(1)) ),
    //                             abs(node1->index(2) - node2->index(2)) );
    
    // double Dijkstra = 0.0;
    
    int dx = abs(node1->index(0) - node2->index(0));
    int dy = abs(node1->index(1) - node2->index(1));
    int dz = abs(node1->index(2) - node2->index(2));
    
    // 方法1：精确3D对角线距离
    int dmin = min(dx, min(dy, dz));
    int dmax = max(dx, max(dy, dz));
    int dmid = dx + dy + dz - dmin - dmax;
    
    double D = 1.0;
    double D2 = sqrt(2.0);   // ≈1.414
    double D3 = sqrt(3.0);   // ≈1.732
    double heu =D3 * dmin + D2 * (dmid - dmin) + D * (dmax - dmid);
    double tie_breaker = 1.0 + 1.0 / 1000.0; // 微小的平局破坏器
    return heu * tie_breaker;
    /* 
    选择可能的启发式函数：
    1. 曼哈顿距离 (Manhattan): |x1-x2| + |y1-y2| + |z1-z2|
    2. 欧几里得距离 (Euclidean): sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2)
    3. 对角线距离 (Diagonal)
    4. 0 (Dijkstra算法)
    
    可选：加上平局破坏器（tie_breaker）
    *
    步骤 1: 完成 AstarPathFinder::getHeu 函数，这是启发式函数
    请在下方编写代码
    *
    */

    return 0;
}

// A*图搜索主函数
void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    rclcpp::Time time_1 = rclcpp::Clock().now();    

    // 初始化 terminatePtr
    terminatePtr = NULL;

    // 将起点和终点坐标转换为栅格索引
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    // 检查起点和终点是否在有效范围内且可行
    if(!isFree(start_idx)){
        RCLCPP_ERROR(rclcpp::get_logger("astar_searcher"), 
                    "[A*] 起点 (%f, %f, %f) 被占用或超出边界！", 
                    start_pt(0), start_pt(1), start_pt(2));
        return;
    }
    if(!isFree(end_idx)){
        RCLCPP_ERROR(rclcpp::get_logger("astar_searcher"), 
                    "[A*] 终点 (%f, %f, %f) 被占用或超出边界！", 
                    end_pt(0), end_pt(1), end_pt(2));
        return;
    }

    // 将起点和终点转换为栅格中心坐标
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    // 初始化起点和终点的网格节点指针
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    // 开集：通过多重映射（multimap）实现，键为f值，值为节点指针
    openSet.clear();
    // currentPtr 表示开集中f值最小的节点
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    // 将起点放入开集
    startPtr -> gScore = 0;  // 起点的g值为0
    startPtr -> fScore = getHeu(startPtr,endPtr);   // 计算f值
    //步骤 1: 完成 AstarPathFinder::getHeu 函数
    startPtr -> id = 1;  // 标记为开集中的节点
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );  // 按f值插入开集
    
    /*
    *
    步骤 2: 在while循环前执行一些准备工作
    请在下方编写代码
    
    提示：
    1. 将起点在GridNodeMap中对应的节点也进行相同的初始化
    2. 准备好邻居节点和边代价的容器
    *
    */
    vector<GridNodePtr> neighborPtrSets;  // 邻居节点集合
    vector<double> edgeCostSets;  // 边代价集合
    
    // 同步GridNodeMap中的起点节点
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->gScore = 0;
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->fScore = startPtr->fScore;
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)]->id = 1;

    // 搜索限制参数
    const int MAX_SEARCH_NODES = 100000;  // 最大搜索节点数
    const double MAX_SEARCH_TIME = 5.0;   // 最大搜索时间（秒）
    int searched_nodes = 0;

    // 主循环：当开集不为空时继续搜索
    while ( !openSet.empty() ){
        currentPtr = openSet.begin()->second;  // 获取f值最小的节点
        openSet.erase(openSet.begin());
        currentPtr -> id = -1;
        searched_nodes++;
        /*
        *
        步骤 3: 从开集中删除f值最小的节点，将其放入闭集
        请在下方编写代码
        
        重要提示!!!
        multimap的用法：
        - openSet.begin() 返回第一个元素的迭代器（f值最小）
        - openSet.erase() 删除指定迭代器的元素
        - 获取节点指针并标记为闭集（id = -1）
        *
        */

        // 检查搜索限制
        if(searched_nodes > MAX_SEARCH_NODES){
            rclcpp::Time time_2 = rclcpp::Clock().now();
            RCLCPP_ERROR(rclcpp::get_logger("astar_searcher"), 
                        "[A*] 搜索节点数超过限制 %d，可能目标不可达！耗时: %f ms", 
                        MAX_SEARCH_NODES, (time_2 - time_1).seconds() * 1000.0);
            return;
        }
        if((rclcpp::Clock().now() - time_1).seconds() > MAX_SEARCH_TIME){
            RCLCPP_ERROR(rclcpp::get_logger("astar_searcher"), 
                        "[A*] 搜索超时 %f 秒，已搜索 %d 个节点！", 
                        MAX_SEARCH_TIME, searched_nodes);
            return;
        }

        // 检查当前节点是否是目标节点
        if( currentPtr->index == goalIdx ){
            rclcpp::Time time_2 = rclcpp::Clock().now();
            terminatePtr = currentPtr;
            RCLCPP_WARN(rclcpp::get_logger("astar_searcher"), 
                       "[A*]{success} A*搜索耗时: %f ms, 路径代价: %f m", 
                       (time_2 - time_1).seconds() * 1000.0, 
                       currentPtr->gScore * resolution);            
            return;
        }
        
        // 获取当前节点的所有邻居节点及其边代价
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //步骤 4: 完成 AstarPathFinder::AstarGetSucc 函数

        /*
        *
        步骤 5: 对节点"n"的所有未展开邻居"m"进行处理
        请在下方编写代码
        *        
        */         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            neighborPtr = neighborPtrSets[i];
            /*
            *
            判断邻居节点是否已被展开
            请在下方编写代码
            
            重要提示!!!
            neighborPtr->id = -1 : 已展开，该节点在闭集中
            neighborPtr->id = 1 : 未展开，该节点在开集中
            neighborPtr->id = 0 : 新节点，既不在开集也不在闭集
            *        
            */
            if(neighborPtr -> id == 0){  // 发现新节点，既不在闭集也不在开集中
                neighborPtr -> gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr -> fScore = neighborPtr -> gScore + getHeu(neighborPtr, endPtr);
                neighborPtr -> cameFrom = currentPtr;
                neighborPtr -> id = 1; // 标记为开集中的节点
                neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );
                /*
                *
                步骤 6: 对于新节点，执行必要的操作，将其加入开集并记录
                请在下方编写代码
                
                提示：
                1. 计算新的gScore = currentPtr->gScore + edgeCostSets[i]
                2. 计算fScore = gScore + getHeu(neighborPtr, endPtr)
                3. 设置neighborPtr的cameFrom指向currentPtr
                4. 标记neighborPtr->id = 1（表示在开集中）
                5. 将neighborPtr插入openSet
                *        
                */
                continue;
            }
            else if(neighborPtr -> id == 1){ // 该节点在开集中，需要判断是否需要更新
                if(neighborPtr -> gScore > currentPtr->gScore + edgeCostSets[i]){ 
                    neighborPtr -> gScore = currentPtr->gScore + edgeCostSets[i];
                    neighborPtr -> fScore = neighborPtr -> gScore + getHeu(neighborPtr, endPtr);
                    neighborPtr -> cameFrom = currentPtr;
                    openSet.erase(neighborPtr->nodeMapIt);
                    neighborPtr -> nodeMapIt = openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );
                }
                /*
                *
                步骤 7: 对于开集中的节点，更新其值，维护开集
                请在下方编写代码
                
                提示：
                1. 计算新的tentative_gScore = currentPtr->gScore + edgeCostSets[i]
                2. 如果tentative_gScore < neighborPtr->gScore，则更新：
                   - 从openSet中删除旧的节点（使用nodeMapIt）
                   - 更新gScore和fScore
                   - 更新cameFrom
                   - 重新插入openSet
                *        
                */
                continue;
            }
            else{  
                
                // 该节点在闭集中（id == -1）
                /*
                *
                请在下方编写代码
                
                提示：闭集中的节点通常不需要处理，直接continue
                但如果你想实现reopening策略，可以在这里添加
                *        
                */
                continue;
            }
        }      
    }
    
    // 如果搜索失败
    rclcpp::Time time_2 = rclcpp::Clock().now();
    if((time_2 - time_1).seconds() > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("astar_searcher"), 
                   "A*路径搜索耗时: %f 秒", (time_2 - time_1).seconds());
}

// 获取搜索到的路径
vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;

    if (terminatePtr == NULL)
        return path;

    // 使用局部变量遍历路径，不要修改 terminatePtr
    GridNodePtr currentPtr = terminatePtr;
    while (currentPtr->cameFrom != NULL)
    {
        gridPath.push_back(currentPtr->cameFrom);
        currentPtr = currentPtr->cameFrom;
    }
    
    /*
    *
    步骤 8: 从当前节点回溯到起点，获取路径上的所有节点
    请在下方编写代码
    
    提示：
    1. 从terminatePtr开始
    2. 循环：将当前节点加入gridPath，然后移动到其cameFrom节点
    3. 直到cameFrom为NULL（到达起点）
    *      
    */

    // 将网格路径转换为坐标路径
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
    
    // 反转路径使其从起点到终点    
    reverse(path.begin(),path.end());

    return path;
}

