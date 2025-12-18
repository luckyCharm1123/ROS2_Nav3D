#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include "sample_waypoints.hpp" // 包含预定义的形状函数（如 circle, eight, point 等）
#include <vector>
#include <deque>
#include <string>
#include <boost/format.hpp>
#include <eigen3/Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std;
using bfmt = boost::format;
using std::placeholders::_1;
// 定义 WaypointGenerator 类，继承自 rclcpp::Node
class WaypointGenerator : public rclcpp::Node
{
public:
    // 构造函数
    WaypointGenerator() : Node("waypoint_generator", 
        // 节点选项：允许未声明的参数（方便从 YAML 文件加载大量动态参数），
        // 并自动从重写（overrides）中声明参数
        rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
    {
        // Publishers
        // --- 初始化发布者 (Publishers) ---
        // 发布 nav_msgs::msg::Path，通常用于给控制器执行
        pub1_ = this->create_publisher<nav_msgs::msg::Path>("waypoints", 50);
        // 发布 geometry_msgs::msg::PoseArray，通常用于 RViz 可视化显示一组箭头
        pub2_ = this->create_publisher<geometry_msgs::msg::PoseArray>("waypoints_vis", 10);

        // Subscribers
        // --- 初始化订阅者 (Subscribers) ---
        // 订阅里程计信息，用于获取机器人当前位置
        sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&WaypointGenerator::odom_callback, this, _1));
        // 订阅目标点（通常来自 RViz 的 "2D Nav Goal" 工具）
        sub2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal", 10, std::bind(&WaypointGenerator::goal_callback, this, _1));
        // 订阅触发信号，用于在特定位置触发预设轨迹
        sub3_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "traj_start_trigger", 10, std::bind(&WaypointGenerator::traj_start_trigger_callback, this, _1));

        // Parameters
        // --- 参数处理 ---
        // 声明并获取 'waypoint_type' 参数，默认为 'manual'
        this->declare_parameter("waypoint_type", "manual");
        this->get_parameter("waypoint_type", waypoint_type_);

        // 初始化状态变量
        is_odom_ready_ = false;
        trigged_time_ = rclcpp::Time(0);

        RCLCPP_INFO(this->get_logger(), "Waypoint Generator Node Started. Type: %s", waypoint_type_.c_str());
    }

private:
    // 定义成员变量：发布者、订阅者
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub1_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub2_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub2_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub3_;
    // 内部逻辑变量
    string waypoint_type_;// 当前生成的轨迹类型
    bool is_odom_ready_; // 里程计是否已就绪
    nav_msgs::msg::Odometry odom_;// 存储最新的里程计数据
    nav_msgs::msg::Path waypoints_;// 当前生成的路径

    std::deque<nav_msgs::msg::Path> waypointSegments_;// 存储分段路径的队列（用于 series 模式）
    rclcpp::Time trigged_time_;// 触发轨迹生成的时间

    // Helper to get Yaw from Quaternion Msg
    // --- 辅助函数：从四元数消息中提取偏航角 (Yaw) ---
    double getYaw(const geometry_msgs::msg::Quaternion &q_msg) {
        tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;
    }
    // --- 核心逻辑：加载指定 ID 的轨迹段 ---
    // 这个函数通常配合 YAML 参数文件使用，读取如 seg0.x, seg0.y 等参数
    void load_seg(int segid, const rclcpp::Time& time_base) {
        // 1. 构造参数前缀 (假设格式为 seg0.yaw)
        std::string prefix = "seg" + std::to_string(segid) + ".";

        RCLCPP_INFO(this->get_logger(), "Getting segment %d", segid);
        
        double yaw = 0.0;
        double time_from_start = 0.0;

        // 2. 获取参数
        // 2. 获取参数：该段轨迹相对于起点的偏航角偏移
        if(!this->get_parameter(prefix + "yaw", yaw)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get param: %syaw", prefix.c_str());
            return;
        }
        
        // --- 修正点 1: 替换 RCLCPP_ASSERT_MSG ---
        // 检查 yaw 是否在合理范围内 (-PI 到 PI)
        if (!((yaw > -3.1499999) && (yaw < 3.14999999))) {
            RCLCPP_ERROR(this->get_logger(), "yaw=%.3f is out of range (-3.15, 3.15)", yaw);
            return; // 或者使用 throw std::runtime_error("Yaw error");
        }
        // 获取参数：该段轨迹应该在触发后多久开始执行
        if(!this->get_parameter(prefix + "time_from_start", time_from_start)){
             RCLCPP_ERROR(this->get_logger(), "Failed to get param: %stime_from_start", prefix.c_str());
             return;
        }

        // --- 修正点 2: 替换 RCLCPP_ASSERT ---
        if (time_from_start < 0.0) {
            RCLCPP_ERROR(this->get_logger(), "time_from_start must be >= 0.0");
            return;
        }
        // 获取参数：该段轨迹的点集 (x, y, z 数组)
        std::vector<double> ptx, pty, ptz;
        this->get_parameter(prefix + "x", ptx);
        this->get_parameter(prefix + "y", pty);
        this->get_parameter(prefix + "z", ptz);

        // --- 修正点 3: 替换关于数组大小的 ASSERT ---
        // 检查点集是否为空或维度不匹配
        if (ptx.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Segment %d points (x) is empty!", segid);
            return;
        }
        if (ptx.size() != pty.size() || ptx.size() != ptz.size()) {
            RCLCPP_ERROR(this->get_logger(), "Segment %d points size mismatch: x(%zu), y(%zu), z(%zu)", 
                segid, ptx.size(), pty.size(), ptz.size());
            return;
        }

        // 3. 生成路径逻辑
        // 3. 生成路径逻辑：坐标变换
        nav_msgs::msg::Path path_msg;
        // 设置该路径段的时间戳 = 基础时间 + 相对开始时间
        path_msg.header.stamp = time_base + rclcpp::Duration::from_seconds(time_from_start);
        // 获取当前机器人的朝向 (Yaw)
        double baseyaw = getYaw(odom_.pose.pose.orientation);
        // 遍历所有点，进行坐标变换（从局部参数坐标 -> 世界坐标）
        for (size_t k = 0; k < ptx.size(); ++k) {
            geometry_msgs::msg::PoseStamped pt;
            // 设置点的朝向：基础朝向 + 参数定义的偏移
            pt.pose.orientation = createQuaternionMsgFromYaw(baseyaw + yaw);
            // 使用 Eigen 进行 2D 旋转变换
            Eigen::Vector2d dp(ptx.at(k), pty.at(k));
            Eigen::Vector2d rdp;
            // 旋转矩阵公式：x' = x*cos(theta) - y*sin(theta), y' = ...
            // 注意这里的角度是 (-baseyaw - yaw)，这通常意味着将局部坐标转换回全局坐标
            rdp.x() = std::cos(-baseyaw-yaw)*dp.x() + std::sin(-baseyaw-yaw)*dp.y();
            rdp.y() =-std::sin(-baseyaw-yaw)*dp.x() + std::cos(-baseyaw-yaw)*dp.y();
            // 加上机器人当前的坐标 (平移变换)
            pt.pose.position.x = rdp.x() + odom_.pose.pose.position.x;
            pt.pose.position.y = rdp.y() + odom_.pose.pose.position.y;
            pt.pose.position.z = ptz.at(k) + odom_.pose.pose.position.z;
            path_msg.poses.push_back(pt);
        }
        // 将生成的这一段路径加入队列
        waypointSegments_.push_back(path_msg);
    }
    // --- 加载多段轨迹 ---
    void load_waypoints(const rclcpp::Time& time_base) {
        int seg_cnt = 0;
        waypointSegments_.clear();
        
        // 尝试获取 segment_cnt，如果没有则默认为 0
        // 获取总段数
        this->get_parameter_or("segment_cnt", seg_cnt, 0);
        
        for (int i = 0; i < seg_cnt; ++i) {
            load_seg(i, time_base);// 加载每一段
            // 简单的时间顺序检查
            if (i > 0) {
                // Time comparison
                rclcpp::Time t_prev = waypointSegments_[i - 1].header.stamp;
                rclcpp::Time t_curr = waypointSegments_[i].header.stamp;
                if(t_prev >= t_curr) {
                    RCLCPP_ERROR(this->get_logger(), "Segment time order error at index %d", i);
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Overall load %zu segments", waypointSegments_.size());
    }
    // --- 发布路径 (nav_msgs::msg::Path) ---
    void publish_waypoints() {
        waypoints_.header.frame_id = "world";// 坐标系为 world
        waypoints_.header.stamp = this->now();
        pub1_->publish(waypoints_);
        
        // Optional: clear poses logic from original was mostly for visualization or one-shot
        // 发布后清空 pose，通常是为了避免重复发送旧数据，或者是单次触发逻辑
        waypoints_.poses.clear();
    }
    // --- 发布可视化 (geometry_msgs::msg::PoseArray) ---
    void publish_waypoints_vis() {
        nav_msgs::msg::Path wp_vis = waypoints_;
        geometry_msgs::msg::PoseArray poseArray;
        poseArray.header.frame_id = "world";
        poseArray.header.stamp = this->now();
        // 将当前机器人位置作为可视化的第一个点
        {
            geometry_msgs::msg::Pose init_pose;
            init_pose = odom_.pose.pose;
            poseArray.poses.push_back(init_pose);
        }
        // 添加路径中的所有点
        for (auto & pose_stamped : waypoints_.poses) {
            poseArray.poses.push_back(pose_stamped.pose);
        }
        pub2_->publish(poseArray);
    }
    // --- 里程计回调函数 ---
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        is_odom_ready_ = true;
        odom_ = *msg;
        // 如果队列中有待发布的轨迹段 (用于 series 模式)
        if (!waypointSegments_.empty()) {
            rclcpp::Time expected_time = waypointSegments_.front().header.stamp;
            // ROS2 time comparison
            // 检查当前时间是否到达该段轨迹的预设执行时间
            if (odom_.header.stamp.sec >= expected_time.seconds()) { // Simply comparing seconds for simplicity, or use full compare
                 // Better comparison:
                 rclcpp::Time odom_time(odom_.header.stamp);
                 if (odom_time >= expected_time) {
                    // 取出队首的路径段
                    waypoints_ = waypointSegments_.front();
                    // 打印调试信息：输出即将发布的点的坐标
                    std::stringstream ss;
                    ss << bfmt("Series send %.3f from start:\n") % trigged_time_.seconds();
                    for (auto& pose_stamped : waypoints_.poses) {
                        ss << bfmt("P[%.2f, %.2f, %.2f] q(%.2f,%.2f,%.2f,%.2f)") %
                                  pose_stamped.pose.position.x % pose_stamped.pose.position.y %
                                  pose_stamped.pose.position.z % pose_stamped.pose.orientation.w %
                                  pose_stamped.pose.orientation.x % pose_stamped.pose.orientation.y %
                                  pose_stamped.pose.orientation.z << std::endl;
                    }
                    RCLCPP_INFO_STREAM(this->get_logger(), ss.str());
                    // 发布并可视化
                    publish_waypoints_vis();
                    publish_waypoints();
                    // 移除已发布的段
                    waypointSegments_.pop_front();
                 }
            }
        }
    }
    // --- 目标点回调函数 (通常处理 RViz 2D Nav Goal) ---
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        trigged_time_ = this->now();// 记录触发时间
        // 重新获取参数，允许在运行时修改 pattern
        // Reload parameter if it changed
        this->get_parameter("waypoint_type", waypoint_type_);
        // 根据不同模式生成路径
        if (waypoint_type_ == string("circle")) {
            waypoints_ = circle();
            publish_waypoints_vis();
            publish_waypoints();
        } else if (waypoint_type_ == string("eight")) {
            waypoints_ = eight();
            publish_waypoints_vis();
            publish_waypoints();
        } else if (waypoint_type_ == string("point")) {
            waypoints_ = point();
            publish_waypoints_vis();
            publish_waypoints();
        } else if (waypoint_type_ == string("series")) {
            load_waypoints(trigged_time_);
        } else if (waypoint_type_ == string("manual-lonely-waypoint")) {
            if (msg->pose.position.z >= 0) {
                geometry_msgs::msg::PoseStamped pt = *msg;
                waypoints_.poses.clear();
                waypoints_.poses.push_back(pt);
                publish_waypoints_vis();
                publish_waypoints();
            } else {
                RCLCPP_WARN(this->get_logger(), "[waypoint_generator] invalid goal in manual-lonely-waypoint mode.");
            }
        } else {
            // --- 默认 Manual 模式（多点累加）---
            // 如果 z > 0，添加一个新点
            if (msg->pose.position.z > 0) {
                geometry_msgs::msg::PoseStamped pt = *msg;
                // 如果是 noyaw 模式，忽略点击的方向，使用当前机器人朝向
                if (waypoint_type_ == string("noyaw")) {
                    double yaw = getYaw(odom_.pose.pose.orientation);
                    pt.pose.orientation = createQuaternionMsgFromYaw(yaw);
                }
                waypoints_.poses.push_back(pt);
                publish_waypoints_vis();
                // 如果 z 在 -1 到 0 之间，表示撤销上一个点
            } else if (msg->pose.position.z > -1.0) {
                if (waypoints_.poses.size() >= 1) {
                    waypoints_.poses.erase(std::prev(waypoints_.poses.end()));
                }
                publish_waypoints_vis();// 只更新可视化，暂不发布 Path
            } else {
                // 如果 z < -1，表示确认发布
                if (waypoints_.poses.size() >= 1) {
                    publish_waypoints_vis();
                    publish_waypoints();
                }
            }
        }
    }
    // --- 触发器回调函数 ---
    // 用于通过外部信号（PoseStamped）触发预设轨迹
    void traj_start_trigger_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!is_odom_ready_) {
            RCLCPP_ERROR(this->get_logger(), "[waypoint_generator] No odom!");
            return;
        }

        RCLCPP_WARN(this->get_logger(), "[waypoint_generator] Trigger!");
        trigged_time_ = odom_.header.stamp;// 以当前里程计时间为准

        this->get_parameter("waypoint_type", waypoint_type_);

        RCLCPP_ERROR_STREAM(this->get_logger(), "Pattern " << waypoint_type_ << " generated!");
        
        if (waypoint_type_ == string("free") || waypoint_type_ == string("point")) {
            waypoints_ = point();
            publish_waypoints_vis();
            publish_waypoints();
        } else if (waypoint_type_ == string("circle")) {
            waypoints_ = circle();
            publish_waypoints_vis();
            publish_waypoints();
        } else if (waypoint_type_ == string("eight")) {
            waypoints_ = eight();
            publish_waypoints_vis();
            publish_waypoints();
        } else if (waypoint_type_ == string("series")) {
            load_waypoints(trigged_time_);
        }
        // Use msg to avoid unused warning
        // 防止未使用变量警告
        (void)msg;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}