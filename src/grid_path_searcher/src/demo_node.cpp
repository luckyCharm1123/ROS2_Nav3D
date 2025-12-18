#include <memory>
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "Astar_searcher.hpp"
#include "JPS_searcher.h"


using namespace std;
using namespace Eigen;

class DemoNode : public rclcpp::Node
{
public:
DemoNode(const std::string& name) : Node(name)
    {
        // === 参数声明并读取 ===

        // 地图参数,点云边界、分辨率
        this->declare_parameter("map.cloud_margin", 0.0);
        this->declare_parameter("map.resolution",   0.2);

        // 地图尺寸
        this->declare_parameter("map.x_size", 50.0);
        this->declare_parameter("map.y_size", 50.0);
        this->declare_parameter("map.z_size", 5.0);

        // 起始点位置
        this->declare_parameter("planning.start_x", 0.0);
        this->declare_parameter("planning.start_y", 0.0);
        this->declare_parameter("planning.start_z", 0.0);

        // 读取参数值
        this->get_parameter("map.cloud_margin", cloud_margin_);
        this->get_parameter("map.resolution",   resolution_);

        this->get_parameter("map.x_size", x_size_);
        this->get_parameter("map.y_size", y_size_);
        this->get_parameter("map.z_size", z_size_);

        this->get_parameter("planning.start_x", start_pt_(0));
        this->get_parameter("planning.start_y", start_pt_(1));
        this->get_parameter("planning.start_z", start_pt_(2));

        // 计算地图边界
        map_lower_ << -x_size_ / 2.0, -y_size_ / 2.0, 0.0;
        map_upper_ << +x_size_ / 2.0, +y_size_ / 2.0, z_size_;

        // 计算分辨率倒数
        inv_resolution_ = 1.0 / resolution_;

        // 计算栅格地图尺寸
        max_x_id_ = static_cast<int>(x_size_ * inv_resolution_);//将结果转换成int类型，也就是点云按照分辨率划分后的栅格数量
        max_y_id_ = static_cast<int>(y_size_ * inv_resolution_);
        max_z_id_ = static_cast<int>(z_size_ * inv_resolution_);

         // === A* / JPS 初始化 ===
        astar_path_finder_ = std::make_unique<AstarPathFinder>();
        astar_path_finder_->initGridMap(
            resolution_, map_lower_, map_upper_, max_x_id_, max_y_id_, max_z_id_);

        jps_path_finder_ = std::make_unique<JPSPathFinder>();
        jps_path_finder_->initGridMap(
            resolution_, map_lower_, map_upper_, max_x_id_, max_y_id_, max_z_id_);
        
        // === ROS2 Publisher / Subscriber ===
        map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "global_map", rclcpp::QoS(1),
            std::bind(&DemoNode::rcvPointCloudCallBack, this, std::placeholders::_1));

        pts_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", rclcpp::QoS(1),
            std::bind(&DemoNode::rcvGoalCallback, this, std::placeholders::_1));

        grid_map_vis_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "grid_map_vis", rclcpp::QoS(1).transient_local());

        grid_path_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "grid_path_vis", rclcpp::QoS(1).transient_local());

        visited_nodes_vis_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "visited_nodes_vis", rclcpp::QoS(1).transient_local());

        RCLCPP_INFO(this->get_logger(), "DemoNode initialized.");
    }
private:
    // 接收路径点回调函数
    void rcvGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 若地图尚未构建完成，则直接返回
        if (!has_map_) return;

        // 提取目标点
        Vector3d target_pt;
        target_pt << msg->pose.position.x,
                     msg->pose.position.y,
                     msg->pose.position.z;

        // 输出接收到目标点信息
        RCLCPP_INFO(this->get_logger(), "[node] receive the planning target");
        pathFinding(start_pt_, target_pt);
    }

    // 接收点云回调函数
    void rcvPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map)
    {
        // 若地图已构建完成，则直接返回
        if (has_map_) return;

        // 将点云消息转换为PCL格式
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud_vis;
        
        // ROS消息格式点云
        sensor_msgs::msg::PointCloud2 map_vis;

        // 转换为PCL点云
        pcl::fromROSMsg(*pointcloud_map, cloud);

        // 若点云为空，直接返回
        if (cloud.points.empty()) return;

        // 遍历点云，设置障碍物
        pcl::PointXYZ pt;
        for (size_t idx = 0; idx < cloud.points.size(); idx++)
        {
            pt = cloud.points[idx];

            // 设置障碍物
            astar_path_finder_->setObs(pt.x, pt.y, pt.z);
            jps_path_finder_->setObs(pt.x, pt.y, pt.z);

            // for visualize only
            Vector3d cor_round = astar_path_finder_->coordRounding(
                Vector3d(pt.x, pt.y, pt.z));
            pt.x = cor_round(0);
            pt.y = cor_round(1);
            pt.z = cor_round(2);
            cloud_vis.points.push_back(pt);
        }

        cloud_vis.width    = cloud_vis.points.size();
        cloud_vis.height   = 1;
        cloud_vis.is_dense = true;

        pcl::toROSMsg(cloud_vis, map_vis);

        map_vis.header.frame_id = "world";
        map_vis.header.stamp = this->now();

        grid_map_vis_pub_->publish(map_vis);

        has_map_ = true;
        RCLCPP_INFO(this->get_logger(), "Received map and built grid map.");
    }

    // === A* / JPS 路径规划 ===
    void pathFinding(const Vector3d start_pt, const Vector3d target_pt)
    {
        // A* 搜索
        RCLCPP_INFO(this->get_logger(), "[Debug] Starting A* search");
        astar_path_finder_->AstarGraphSearch(start_pt, target_pt);

        // 获取路径和访问过的节点
        RCLCPP_INFO(this->get_logger(), "[Debug] Getting A* path");
        auto grid_path     = astar_path_finder_->getPath();
        RCLCPP_INFO(this->get_logger(), "[Debug] Getting A* visited nodes");
        auto visited_nodes = astar_path_finder_->getVisitedNodes();

        // 可视化结果
        RCLCPP_INFO(this->get_logger(), "[Debug] Visualizing A* path");
        visGridPath(grid_path, false);
        RCLCPP_INFO(this->get_logger(), "[Debug] Visualizing A* visited nodes");
        visVisitedNode(visited_nodes);

        // 重置地图以供下一次调用
        RCLCPP_INFO(this->get_logger(), "[Debug] Resetting A* grids");
        astar_path_finder_->resetUsedGrids();

        // _use_jps = 0 -> 不使用JPS
        // _use_jps = 1 -> 使用JPS
        // 你只需要更改#define的值即可
        #define _use_jps 1
        #if _use_jps
        {
            RCLCPP_INFO(this->get_logger(), "[Debug] Starting JPS search");
            jps_path_finder_->JPSGraphSearch(start_pt, target_pt);

            RCLCPP_INFO(this->get_logger(), "[Debug] Getting JPS path");
            auto jps_grid_path     = jps_path_finder_->getPath();
            RCLCPP_INFO(this->get_logger(), "[Debug] Getting JPS visited nodes");
            auto jps_visited_nodes = jps_path_finder_->getVisitedNodes();

            RCLCPP_INFO(this->get_logger(), "[Debug] Visualizing JPS path");
            visGridPath(jps_grid_path, true);
            RCLCPP_INFO(this->get_logger(), "[Debug] Visualizing JPS visited nodes");
            visVisitedNode(jps_visited_nodes);

            RCLCPP_INFO(this->get_logger(), "[Debug] Resetting JPS grids");
            jps_path_finder_->resetUsedGrids();
            RCLCPP_INFO(this->get_logger(), "[Debug] Path finding completed");
        }
        #endif
    }

        // === RViz 可视化 ===
    void visGridPath(const vector<Vector3d> &nodes, bool is_use_jps)
    {
        // 可视化路径
        visualization_msgs::msg::Marker node_vis;
        node_vis.header.frame_id = "world";
        node_vis.header.stamp    = this->now();

        // 设置命名空间
        if (is_use_jps)
            node_vis.ns = "demo_node/jps_path";
        else
            node_vis.ns = "demo_node/astar_path";

        // 设置类型、动作和ID
        node_vis.type   = visualization_msgs::msg::Marker::CUBE_LIST;
        node_vis.action = visualization_msgs::msg::Marker::ADD;
        node_vis.id     = 0;

        // 设置姿态
        node_vis.pose.orientation.x = 0.0;
        node_vis.pose.orientation.y = 0.0;
        node_vis.pose.orientation.z = 0.0;
        node_vis.pose.orientation.w = 1.0;

        if (is_use_jps) {
            // 设置颜色为红色
            node_vis.color.a = 1.0;
            node_vis.color.r = 1.0;
            node_vis.color.g = 0.0;
            node_vis.color.b = 0.0;
        } else {
            // 设置颜色为绿色
            node_vis.color.a = 1.0;
            node_vis.color.r = 0.0;
            node_vis.color.g = 0.0;
            node_vis.color.b = 0.0;
        }

        // 设置缩放比例
        node_vis.scale.x = resolution_;
        node_vis.scale.y = resolution_;
        node_vis.scale.z = resolution_;

        // 设置生命周期为永久
        node_vis.lifetime = rclcpp::Duration::from_seconds(0);  // 0 表示永久存在

        // 设置路径点
        geometry_msgs::msg::Point pt;
        for (const auto & coord : nodes)
        {
            pt.x = coord(0);
            pt.y = coord(1);
            pt.z = coord(2);
            node_vis.points.push_back(pt);
        }

        // 发布路径可视化消息
        grid_path_vis_pub_->publish(node_vis);
    }

    // === RViz 可视化 ===
    void visVisitedNode(const vector<Vector3d> &nodes)
    {
        // 可视化访问过的节点
        visualization_msgs::msg::Marker node_vis;
        node_vis.header.frame_id = "world";
        node_vis.header.stamp    = this->now();
        node_vis.ns              = "demo_node/expanded_nodes";
        node_vis.type            = visualization_msgs::msg::Marker::CUBE_LIST;
        node_vis.action          = visualization_msgs::msg::Marker::ADD;
        node_vis.id              = 0;

        node_vis.pose.orientation.x = 0.0;
        node_vis.pose.orientation.y = 0.0;
        node_vis.pose.orientation.z = 0.0;
        node_vis.pose.orientation.w = 1.0;

        node_vis.color.a = 0.5;
        node_vis.color.r = 0.0;
        node_vis.color.g = 0.0;
        node_vis.color.b = 1.0;

        node_vis.scale.x = resolution_;
        node_vis.scale.y = resolution_;
        node_vis.scale.z = resolution_;

        // 设置生命周期为永久
        node_vis.lifetime = rclcpp::Duration::from_seconds(0);  // 0 表示永久存在

        geometry_msgs::msg::Point pt;
        for (const auto & coord : nodes)
        {
            pt.x = coord(0);
            pt.y = coord(1);
            pt.z = coord(2);
            node_vis.points.push_back(pt);
        }

        visited_nodes_vis_pub_->publish(node_vis);
    }

    // === 成员变量 ===
    // 地图 / 分辨率
    double resolution_{0.2}, inv_resolution_{5.0}, cloud_margin_{0.0};
    double x_size_{50.0}, y_size_{50.0}, z_size_{5.0};

    bool has_map_{false};

    Vector3d start_pt_{0.0, 0.0, 0.0};
    Vector3d map_lower_{0.0, 0.0, 0.0};
    Vector3d map_upper_{0.0, 0.0, 0.0};

    int max_x_id_{0}, max_y_id_{0}, max_z_id_{0};

    // ROS2 通信
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pts_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr grid_map_vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr grid_path_vis_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visited_nodes_vis_pub_;

    // 路径搜索器
    std::unique_ptr<AstarPathFinder> astar_path_finder_;
    std::unique_ptr<JPSPathFinder>   jps_path_finder_;

};

int main(int argc, char** argv)
{
    // 初始化ROS2节点
    rclcpp::init(argc, argv);

    // 创建DemoNode节点
    auto demo_node = std::make_shared<DemoNode>("demo_node");

    // 运行节点
    rclcpp::spin(demo_node);

    // 关闭ROS2节点
    rclcpp::shutdown();
    return 0;
}


