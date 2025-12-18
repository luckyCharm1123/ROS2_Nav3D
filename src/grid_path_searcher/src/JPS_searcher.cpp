#include "JPS_searcher.h"
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace Eigen;

// 获取当前节点的后继节点（Successors），这是 JPS 的核心逻辑之一
// 根据 JPS 的剪枝规则（Pruning Rules）寻找“跳点”（Jump Points）
inline void JPSPathFinder::JPSGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{
    // 清空后继节点和边代价集合
    neighborPtrSets.clear();
    edgeCostSets.clear();
    // 计算方向向量的 L1 范数 (曼哈顿距离)，用于判断移动类型：
    // norm1 = 1: 直线移动 (6个方向)
    // norm1 = 2: 2D平面斜向移动 (12个方向)
    // norm1 = 3: 3D立体斜向移动 (8个方向)
    const int norm1 = abs(currentPtr->dir(0)) + abs(currentPtr->dir(1)) + abs(currentPtr->dir(2));

    // jn3d 是一个预计算的查找表，用于根据 JPS 的剪枝规则快速获取需要检查的邻居方向
    // num_neib: 自然邻居 (Natural Neighbors) 的数量（沿运动方向自然延伸的邻居）
    // num_fneib: 强迫邻居 (Forced Neighbors) 的数量（由于障碍物存在而被迫需要检查的邻居）
    int num_neib  = jn3d->nsz[norm1][0];
    int num_fneib = jn3d->nsz[norm1][1];
    // 计算当前方向在查找表中的索引 ID，三维坐标系中的索引 ID 是一个 3D 向量的三元组，
    int id = (currentPtr->dir(0) + 1) + 3 * (currentPtr->dir(1) + 1) + 9 * (currentPtr->dir(2) + 1);
    // 遍历所有可能的扩展方向（包括自然邻居和潜在的强迫邻居方向）
    for( int dev = 0; dev < num_neib + num_fneib; ++dev) {
        // 计算当前方向的邻居索引和扩展方向
        Vector3i neighborIdx;
        Vector3i expandDir;
        // 处理自然邻居 (Natural Neighbors)
        if( dev < num_neib ) {
            // 从表中获取扩展方向
            expandDir(0) = jn3d->ns[id][0][dev];
            expandDir(1) = jn3d->ns[id][1][dev];
            expandDir(2) = jn3d->ns[id][2][dev];
            // 执行跳跃操作 (jump)，如果跳跃失败（遇到障碍或边界），则跳过
            if( !jump(currentPtr->index, expandDir, neighborIdx) )  
                continue;
        }
        // 处理强迫邻居 (Forced Neighbors)
        else {
            // 检查产生强迫邻居的障碍物是否存在
            // f1 存储的是用于检测障碍物的偏移量
            int nx = currentPtr->index(0) + jn3d->f1[id][0][dev - num_neib];
            int ny = currentPtr->index(1) + jn3d->f1[id][1][dev - num_neib];
            int nz = currentPtr->index(2) + jn3d->f1[id][2][dev - num_neib];
            
            if( isOccupied(nx, ny, nz) ) {
                expandDir(0) = jn3d->f2[id][0][dev - num_neib];
                expandDir(1) = jn3d->f2[id][1][dev - num_neib];
                expandDir(2) = jn3d->f2[id][2][dev - num_neib];
                
                if( !jump(currentPtr->index, expandDir, neighborIdx) ) 
                    continue;
            }
            else
                continue;
        }
        
        // 边界检查：确保 neighborIdx 在有效范围内
        if(neighborIdx(0) < 0 || neighborIdx(0) >= GLX_SIZE ||
           neighborIdx(1) < 0 || neighborIdx(1) >= GLY_SIZE ||
           neighborIdx(2) < 0 || neighborIdx(2) >= GLZ_SIZE)
            continue;
        
        // 获取跳点的节点指针
        GridNodePtr nodePtr = GridNodeMap[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
        // 更新该节点的父节点进入方向（这在 JPS 中很重要，决定了下一次扩展的规则）
        nodePtr->dir = expandDir;
        // 将找到的跳点加入后继集合
        neighborPtrSets.push_back(nodePtr);
        // 计算当前节点到跳点的欧几里得距离作为边代价 (g值增量)
        edgeCostSets.push_back(
            sqrt(
            (neighborIdx(0) - currentPtr->index(0)) * (neighborIdx(0) - currentPtr->index(0)) +
            (neighborIdx(1) - currentPtr->index(1)) * (neighborIdx(1) - currentPtr->index(1)) +
            (neighborIdx(2) - currentPtr->index(2)) * (neighborIdx(2) - currentPtr->index(2))   ) 
            );
    }
}
// 跳跃函数：从当前索引出发，沿指定方向跳跃，直到遇到障碍物、边界或找到跳点
bool JPSPathFinder::jump(const Vector3i & curIdx, const Vector3i & expDir, Vector3i & neiIdx)
{
    // 向前迈一步，向量expDir 表示当前索引到跳点之间的方向
    neiIdx = curIdx + expDir;

    // 1. 检查边界和障碍物：如果不是空闲的，跳跃失败
    if( !isFree(neiIdx) )
        return false;
    // 2. 目标检查：如果到达目标点，跳跃成功，当前点就是跳点
    if( neiIdx == goalIdx )
        return true;
    // 3. 强迫邻居检查：如果当前节点有强迫邻居，跳跃成功，当前点就是跳点
    if( hasForced(neiIdx, expDir) )
        return true;
    // 4. 递归逻辑（用于对角线移动）：
    // 如果是斜向移动，JPS 规则要求同时检查分量方向（直线方向）是否能产生跳点。
    // 例如：向东北移动，需要检查向东走或向北走是否会遇到跳点。
    const int id = (expDir(0) + 1) + 3 * (expDir(1) + 1) + 9 * (expDir(2) + 1);
    const int norm1 = abs(expDir(0)) + abs(expDir(1)) + abs(expDir(2));
    int num_neib = jn3d->nsz[norm1][0];// 获取分解后的直线方向数量
    // 遍历分量方向进行递归检查
    for( int k = 0; k < num_neib - 1; ++k ){
        Vector3i newNeiIdx;
        Vector3i newDir(jn3d->ns[id][0][k], jn3d->ns[id][1][k], jn3d->ns[id][2][k]);
        // 如果分量方向上发现了跳点，则当前点也必须标记为跳点
        if( jump(neiIdx, newDir, newNeiIdx) ) 
            return true;
    }
    // 5. 继续在主方向上递归前进
    return jump(neiIdx, expDir, neiIdx);
}
// 检查当前节点是否有强迫邻居 (Forced Neighbors)
inline bool JPSPathFinder::hasForced(const Vector3i & idx, const Vector3i & dir)
{
    // 计算移动维度的 L1 范数 (曼哈顿距离)
    int norm1 = abs(dir(0)) + abs(dir(1)) + abs(dir(2));
    // 计算当前方向在查找表中的索引 ID，三维向量转换为索引 ID
    int id    = (dir(0) + 1) + 3 * (dir(1) + 1) + 9 * (dir(2) + 1);

    switch(norm1){
        case 1:
            // 1-d move, check 8 neighbors
            // 直线移动：检查 8 个垂直方向的邻居
            // 这里的逻辑是检查与运动方向垂直的格子是否有障碍物，且该障碍物旁边是空的
            for( int fn = 0; fn < 8; ++fn ){
                //原始坐标+具坐标加上 JPS 的强制邻居向量，即强制邻居向量
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                // 这里的逻辑简化了，通常检测强迫邻居需要判断：障碍物存在 AND 障碍物后方可通过
                // 但这里只判断了 isOccupied，可能具体的逻辑封装在 lookup table (f1) 的设计里了
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        case 2:
            // 2-d move, check 8 neighbors
            // 2D 斜向移动：检查相关的 8 个邻居
            for( int fn = 0; fn < 8; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        case 3:
            // 3-d move, check 6 neighbors
            // 3D 斜向移动：检查 6 个邻居
            for( int fn = 0; fn < 6; ++fn ){
                int nx = idx(0) + jn3d->f1[id][0][fn];
                int ny = idx(1) + jn3d->f1[id][1][fn];
                int nz = idx(2) + jn3d->f1[id][2][fn];
                if( isOccupied(nx, ny, nz) )
                    return true;
            }
            return false;

        default:
            return false;
    }
}

inline bool JPSPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool JPSPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool JPSPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool JPSPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

void JPSPathFinder::JPSGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    rclcpp::Time time_1 = rclcpp::Clock().now();    
    
    RCLCPP_INFO(rclcpp::get_logger("JPS_searcher"), "[JPS Debug] Step 1: Initialize terminatePtr");
    // 初始化 terminatePtr
    terminatePtr = NULL;
    
    RCLCPP_INFO(rclcpp::get_logger("JPS_searcher"), "[JPS Debug] Step 2: Convert coordinates to indices");
    // --- 初始化部分 ---
    // 将起点和终点的物理坐标转换为网格索引
    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    // 检查起点和终点是否在有效范围内且可行
    if(!isFree(start_idx)){
        RCLCPP_ERROR(rclcpp::get_logger("JPS_searcher"), 
                    "[JPS] 起点 (%f, %f, %f) 被占用或超出边界！", 
                    start_pt(0), start_pt(1), start_pt(2));
        return;
    }
    if(!isFree(end_idx)){
        RCLCPP_ERROR(rclcpp::get_logger("JPS_searcher"), 
                    "[JPS] 终点 (%f, %f, %f) 被占用或超出边界！", 
                    end_pt(0), end_pt(1), end_pt(2));
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("JPS_searcher"), "[JPS Debug] Step 3: Convert indices to coordinates");
    //position of start_point and end_point
    // 对齐坐标到网格中心，在A*算法中实现了，这里直接调用
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);
    
    RCLCPP_INFO(rclcpp::get_logger("JPS_searcher"), "[JPS Debug] Step 4: Create start and end nodes");
    // 初始化起点和终点节点
    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);
    //openSet is the open_list implemented through multimap in STL library
    // openSet 是 Open List，使用 multimap 实现（自动根据键值 fScore 排序）
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    // currentPtr 表示 f(n) 最小的节
    //初始化为空
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    // 将起点加入 Open List
    startPtr -> gScore = 0;
    // fScore = gScore + hScore
    //getHeu是启发式函数，这里调用的是 JPS 启发式函数，返回的是 f(n)
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    // STEP 1: 计算启发式代价 (H值)
    // 获取启发式函数，返回 f(n)
    //id is the node id, starting from 1
    // 标记节点状态 (1: 在OpenList中, -1: 未访问, 0: 在ClosedList中，具体看你的定义)
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    /*
    * STEP 2 : 循环前的准备工作
    * 通常这里需要重置整个 GridNodeMap，将所有节点的 id 设为 0 或 -1，
    * 清空之前的父节点指针等，防止上一次搜索的残留数据影响本次搜索。
    */
    resetUsedGrids();
    
    // 同步GridNodeMap中的起点节点，防止重复访问
    GridNodePtr startNodeInMap = GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)];
    startNodeInMap->id = 1;
    startNodeInMap->gScore = 0;
    startNodeInMap->fScore = startPtr->fScore;
    startNodeInMap->cameFrom = NULL;

    double tentative_gScore = 0.0;
    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;
    
    // 搜索限制参数
    const int MAX_SEARCH_NODES = 100000;  // 最大搜索节点数
    const double MAX_SEARCH_TIME = 10.0;   // 最大搜索时间（秒）
    int searched_nodes = 0;
    
    // this is the main loop
    // --- 主循环 ---
    while ( !openSet.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
       /*
        * STEP 3: 从 Open Set 中取出 f(n) 最小的节点作为 currentPtr
        * 并将其从 Open Set 中移除，放入 Closed Set。
        * * 提示：
        * currentPtr = openSet.begin()->second;
        * openSet.erase(openSet.begin());
        * currentPtr->id = -1; // 或者其他表示“已扩展/Closed”的值
        */
        currentPtr = openSet.begin()->second;
        openSet.erase(openSet.begin());
        currentPtr->id = -1;
        searched_nodes++;
        
        // 检查搜索限制
        if(searched_nodes > MAX_SEARCH_NODES){
            rclcpp::Time time_2 = rclcpp::Clock().now();
            RCLCPP_ERROR(rclcpp::get_logger("JPS_searcher"), 
                        "[JPS] 搜索节点数超过限制 %d，可能目标不可达！耗时: %f ms", 
                        MAX_SEARCH_NODES, (time_2 - time_1).seconds() * 1000.0);
            return;
        }
        if((rclcpp::Clock().now() - time_1).seconds() > MAX_SEARCH_TIME){
            RCLCPP_ERROR(rclcpp::get_logger("JPS_searcher"), 
                        "[JPS] 搜索超时 %f 秒，已搜索 %d 个节点！", 
                        MAX_SEARCH_TIME, searched_nodes);
            return;
        }
        
        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            rclcpp::Time time_2 = rclcpp::Clock().now();
            // 记录终点指针用于回溯路径
            terminatePtr = currentPtr;
            RCLCPP_WARN(rclcpp::get_logger("JPS_searcher"), "[JPS]{sucess} Time in JPS is %f ms, path cost if %f m", (time_2 - time_1).seconds() * 1000.0, currentPtr->gScore * resolution );    
            return;
        }
        //get the succetion
        // 扩展当前节点：获取 JPS 后继跳点
        JPSGetSucc(currentPtr, neighborPtrSets, edgeCostSets); //we have done it for you
        
        /*
        *
        *
        STEP 4:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */    
        /*
        * STEP 4: 遍历所有邻居节点
        */     
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : unexpanded
            neighborPtrSets[i]->id = 1 : expanded, equal to this node is in close set
            *        
            */
           // 注意：你代码里的 neighborPtr 并没有赋值！
            // 这里应该加上： neighborPtr = neighborPtrSets[i];
            
            /*
            * 判断邻居状态
            * neighborPtr->id == -1 : 未被访问过 (Unexpanded) - 注意这里的id定义可能和上面有些混淆，需根据头文件确认
            * neighborPtr->id == 1  : 在 Open List 中
            * 通常还有一种状态是 Closed List
            */
            neighborPtr = neighborPtrSets[i];
            tentative_gScore = currentPtr->gScore + edgeCostSets[i];
            if(neighborPtr -> id != 1){ //discover a new node
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
               /*
                * STEP 6: 处理新节点
                * 1. 计算 gScore = currentPtr->gScore + edgeCostSets[i]
                * 2. 计算 fScore = gScore + Heuristic
                * 3. 记录父节点: neighborPtr->cameFrom = currentPtr
                * 4. 将其加入 Open Set
                * 5. 标记 id = 1
                */
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->nodeMapIt = openSet.insert(std::make_pair(neighborPtr->fScore, neighborPtr));
                neighborPtr->id = 1;
                continue;
            }
            else if(tentative_gScore <= neighborPtr-> gScore){ //in open set and need update//
                // 节点已在 Open Set 中，但发现了更优路径
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                /*
                * STEP 7: 更新已存在的节点
                * 1. 更新 gScore 和 fScore
                * 2. 更新父节点
                * 3. (对于 multiset/multimap) 你可能需要先删除旧的元素，再插入新的 (因为 key 变了)
                */
                // 如果父节点改变了，更新 JPS 特有的进入方向 (direction)
                // 这部分代码是 JPS 特有的，因为父节点改变意味着到达该点的方向变了，进而影响后续的跳点搜索

                
                // if change its parents, update the expanding direction 
                //THIS PART IS ABOUT JPS, you can ignore it when you do your Astar work
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->gScore = tentative_gScore;
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
                // Because the fScore is changed, we need to update its position in the multimap
                // 由于 fScore 变了，需要更新在 multimap 中的位置
                // So we need to remove it first and then insert it again
                // 所以先删除再插入
                // Find the iterator to the element to be updated
                openSet.erase(neighborPtr->nodeMapIt);
                neighborPtr->nodeMapIt = openSet.insert(std::make_pair(neighborPtr->fScore, neighborPtr));
                for(int i = 0; i < 3; i++){
                    neighborPtr->dir(i) = neighborPtr->index(i) - currentPtr->index(i);
                    if( neighborPtr->dir(i) != 0)
                        neighborPtr->dir(i) /= abs( neighborPtr->dir(i) );
                }
            }      
        }
    }
    //if search fails
    rclcpp::Time time_2 = rclcpp::Clock().now();
    if((time_2 - time_1).seconds() > 0.1)
        RCLCPP_WARN(rclcpp::get_logger("JPS_searcher"), "Time consume in JPS path finding is %f", (time_2 - time_1).seconds() );
}