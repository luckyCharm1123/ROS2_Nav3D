#ifndef _ASTAR_SEARCHER_H_
#define _ASTAR_SEARCHER_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <vector>
#include <map>

// 定义无穷大
constexpr double inf = 1e10;

struct GridNode
{
    int id;
    Eigen::Vector3d coord;  // 节点坐标
    Eigen::Vector3i index;  // 节点索引
    Eigen::Vector3i dir;    // 方向
    double gScore, fScore;  // g值和f值
    GridNode* cameFrom;     // 父节点指针
    std::multimap<double, GridNode*>::iterator nodeMapIt;  // 在openSet中的迭代器

    // 构造函数
    GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
        id = 0;
        index = _index;
        coord = _coord;
        dir = Eigen::Vector3i::Zero();
        gScore = inf;
        fScore = inf;
        cameFrom = NULL;
    }

    // 默认构造函数
    GridNode(){
        id = 0;
        dir = Eigen::Vector3i::Zero();
        gScore = inf;
        fScore = inf;
        cameFrom = NULL;
    }
    // 析构函数
    ~GridNode(){};
};
typedef GridNode* GridNodePtr;

class AstarPathFinder
{
protected:
    // 地图参数
    uint8_t * data;  // 占用栅格数据
    GridNodePtr *** GridNodeMap;  // 三维网格节点指针数组
    Eigen::Vector3i goalIdx;  // 目标索引
    int GLX_SIZE, GLY_SIZE, GLZ_SIZE;  // 栅格尺寸
    int GLXYZ_SIZE, GLYZ_SIZE;  // 总栅格数

    double resolution, inv_resolution;  // 分辨率及其倒数
    double gl_xl, gl_yl, gl_zl;  // 地图下界
    double gl_xu, gl_yu, gl_zu;  // 地图上界

    GridNodePtr terminatePtr;  // 终止节点指针
    std::multimap<double, GridNodePtr> openSet;  // 开集

    // 内部函数
    // 计算从起点到终点的路径
    void AstarGetSucc(GridNodePtr currentPtr, std::vector<GridNodePtr> & neighborPtrSets, 
                      std::vector<double> & edgeCostSets);        

    // 重构路径                  
    double getHeu(GridNodePtr node1, GridNodePtr node2);
    // 重置节点
    void resetGrid(GridNodePtr ptr);
    // 释放内存
    bool isOccupied(const Eigen::Vector3i & index) const;
    // 释放内存
    bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
    
    // 释放内存
    bool isFree(const Eigen::Vector3i & index) const;
    // 释放内存
    bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
    
    //转换函数
    Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
    Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);

public:

    // 构造函数与析构函数
    AstarPathFinder(){};
    ~AstarPathFinder(){};
    
    // 初始化栅格地图
    void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, 
                     Eigen::Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id);
    
    // 重置已使用的栅格
    void resetUsedGrids();

    // 设置障碍物
    void setObs(const double coord_x, const double coord_y, const double coord_z);

    // 坐标舍入
    Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);

    // A*图搜索
    void AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

    // 获取路径
    std::vector<Eigen::Vector3d> getPath();
    
    // 获取已访问节点
    std::vector<Eigen::Vector3d> getVisitedNodes();
};

#endif
