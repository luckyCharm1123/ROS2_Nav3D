#ifndef SAMPLE_WAYPOINTS_HPP
#define SAMPLE_WAYPOINTS_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 辅助函数：ROS2 中没有直接的 createQuaternionMsgFromYaw

/**
 * @brief 辅助函数：将欧拉角中的偏航角 (Yaw) 转换为 ROS 2 消息格式的四元数 (Quaternion)
 * * 在 ROS 1 中通常有 tf::createQuaternionMsgFromYaw，但在 ROS 2 中通常需要使用 tf2 库手动转换。
 * * @param yaw 偏航角（弧度制）
 * @return geometry_msgs::msg::Quaternion 四元数消息
 */
inline geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}
/**
 * @brief 生成一组特定的折线/直线路径点 (Point/Line Trajectory)
 * * 该函数生成一系列在固定高度上的点，形成一个类似 U 型或特定走向的折线轨迹。
 * * @return nav_msgs::msg::Path 包含一系列 PoseStamped 的路径消息
 */
inline nav_msgs::msg::Path point()
{
    nav_msgs::msg::Path waypoints;
    geometry_msgs::msg::PoseStamped pt;
    // 初始化所有点的朝向为 0 度 (正东方向)
    pt.pose.orientation = createQuaternionMsgFromYaw(0.0);
    // 固定的 Z 轴高度
    double h = 1.0;
    // 坐标缩放因子，用于放大整体轨迹
    double scale = 7.0;
    // Lambda 表达式：用于简化“设置坐标并推入向量”的重复代码
    auto push_pt = [&](double x, double y) {
        pt.pose.position.x = x;
        pt.pose.position.y = y;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);
    };
    // --- 开始定义路径点 ---
    // 下面的坐标点大致形成一个向右延伸，然后向左上方折回的路径
    push_pt(scale * 2.0, scale * 0.0);// 起始段
    push_pt(scale * 4.0, scale * 0.0);
    push_pt(scale * 5.0, scale * 0.25);// 开始转弯
    push_pt(scale * 5.3, scale * 0.5);// 最远端顶点
    push_pt(scale * 5.0, scale * 0.75);
    push_pt(scale * 4.0, scale * 1.0);//折返
    push_pt(scale * 2.0, scale * 1.0);
    push_pt(scale * 0.0, scale * 1.0);// 终点

    return waypoints;
}

// Circle trajectory
/**
 * @brief 生成类似圆形的循环路径点 (Circle Trajectory)
 * * 实际上，根据坐标来看，这更像是一个在两个端点之间往复的三角形或菱形路径，循环两次。
 * * @return nav_msgs::msg::Path 路径消息
 */
inline nav_msgs::msg::Path circle()
{
    double h = 1.0;// 固定高度
    double scale = 5.0;// 缩放因子
    nav_msgs::msg::Path waypoints;
    geometry_msgs::msg::PoseStamped pt;
    pt.pose.orientation = createQuaternionMsgFromYaw(0.0);
    
    auto push_pt = [&](double x, double y) {
        pt.pose.position.x = x;
        pt.pose.position.y = y;
        pt.pose.position.z = h;
        waypoints.poses.push_back(pt);
    };

    // Loop logic copied from original
    // 循环逻辑：将相同的路径模式重复生成 2 次
    for(int k=0; k<2; k++) { // Repeat pattern logic simplied or kept same
        // 以下坐标定义了一个往复的路径图案
         push_pt(2.5 * scale, -1.2 * scale);
         push_pt(5.0 * scale, -2.4 * scale);
         push_pt(5.0 * scale,  0.0 * scale);// 这里回到 Y=0
         push_pt(2.5 * scale, -1.2 * scale);// 折返
         push_pt(0.0 * scale, -2.4 * scale);
         push_pt(0.0 * scale,  0.0 * scale);// 回到原点
    }
    // Note: The original logic had repetitive blocks, I kept the structure essentially same via calls

    return waypoints;
}

// Figure 8 trajectory
inline nav_msgs::msg::Path eight()
{
    double offset_x = 0.0;// X 轴偏移量
    double offset_y = 0.0;// Y 轴偏移量
    double r = 10.0;// 半径/步长基准
    double h = 2.0;// 基础高度基准
    nav_msgs::msg::Path waypoints;
    geometry_msgs::msg::PoseStamped pt;
    pt.pose.orientation = createQuaternionMsgFromYaw(0.0);    

    // First loop
    // --- 第一圈循环 (First loop) ---
    // 这一圈的高度在 h/2 和 h 之间变化
    // 轨迹逻辑：先向右下方移动，绕圈，交叉回到中心，再绕另一侧
    pt.pose.position.x =  r + offset_x;     pt.pose.position.y = -r + offset_y; pt.pose.position.z =  h/2; waypoints.poses.push_back(pt);      
    pt.pose.position.x =  r*2 + offset_x*2; pt.pose.position.y =  0;            pt.pose.position.z =  h;   waypoints.poses.push_back(pt);  
    pt.pose.position.x =  r*3 + offset_x*3; pt.pose.position.y =  r;            pt.pose.position.z =  h/2; waypoints.poses.push_back(pt);  
    pt.pose.position.x =  r*4 + offset_x*4; pt.pose.position.y =  0;            pt.pose.position.z =  h;   waypoints.poses.push_back(pt);       
    pt.pose.position.x =  r*3 + offset_x*3; pt.pose.position.y = -r;            pt.pose.position.z =  h/2; waypoints.poses.push_back(pt);      
    pt.pose.position.x =  r*2 + offset_x*2; pt.pose.position.y =  0;            pt.pose.position.z =  h;   waypoints.poses.push_back(pt);  
    pt.pose.position.x =  r + offset_x*2;   pt.pose.position.y =  r;            pt.pose.position.z =  h/2; waypoints.poses.push_back(pt);  
    pt.pose.position.x =  0  + offset_x;    pt.pose.position.y =  0;            pt.pose.position.z =  h;   waypoints.poses.push_back(pt);
    
    // Second loop
    // --- 第二圈循环 (Second loop) ---
    // 这一圈的逻辑与第一圈类似，但是高度更高。
    // 高度在 h (2.0) 和 h*1.5 (3.0) 之间变化，形成螺旋上升的效果。
    pt.pose.position.x =  r + offset_x;     pt.pose.position.y = -r; pt.pose.position.z =  h / 2 * 3; waypoints.poses.push_back(pt);      
    pt.pose.position.x =  r*2 + offset_x*2; pt.pose.position.y =  0; pt.pose.position.z =  h;         waypoints.poses.push_back(pt);  
    pt.pose.position.x =  r*3 + offset_x*3; pt.pose.position.y =  r; pt.pose.position.z =  h / 2 * 3; waypoints.poses.push_back(pt);  
    pt.pose.position.x =  r*4 + offset_x*4; pt.pose.position.y =  0; pt.pose.position.z =  h;         waypoints.poses.push_back(pt);       
    pt.pose.position.x =  r*3 + offset_x*3; pt.pose.position.y = -r; pt.pose.position.z =  h / 2 * 3; waypoints.poses.push_back(pt);      
    pt.pose.position.x =  r*2 + offset_x*2; pt.pose.position.y =  0; pt.pose.position.z =  h;         waypoints.poses.push_back(pt);  
    pt.pose.position.x =  r + offset_x;     pt.pose.position.y =  r + offset_y; pt.pose.position.z =  h / 2 * 3; waypoints.poses.push_back(pt);  
    pt.pose.position.x =  0;                pt.pose.position.y =  0; pt.pose.position.z =  h;         waypoints.poses.push_back(pt);  

    return waypoints;   
}  
#endif