#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class RandomComplexGenerator : public rclcpp::Node
{
public:
    RandomComplexGenerator(const std::string& name) : Node(name){
        // 声明并获取参数
        //初始位置设定
        this->declare_parameter("init_state_x", 0.0);
        this->declare_parameter("init_state_y", 0.0);
        //地图大小
        this->declare_parameter("map.x_size", 50.0);
        this->declare_parameter("map.y_size", 50.0);
        this->declare_parameter("map.z_size", 5.0);
        //障碍物数量及分辨率
        this->declare_parameter("map.obs_num", 30);
        this->declare_parameter("map.circle_num", 30);
        this->declare_parameter("map.resolution", 0.2);
        //地面生成参数
        this->declare_parameter("map.generate_ground", 1);
        this->declare_parameter("map.max_ground_slope", 0.3);
         // 读取参数：柱状障碍物形状参数
        this->declare_parameter("ObstacleShape.lower_rad", 0.3);
        this->declare_parameter("ObstacleShape.upper_rad", 0.8);
        this->declare_parameter("ObstacleShape.lower_hei", 3.0);
        this->declare_parameter("ObstacleShape.upper_hei", 7.0);
        // 读取参数：圆形障碍物形状参数
        this->declare_parameter("CircleShape.lower_circle_rad", 0.3);
        this->declare_parameter("CircleShape.upper_circle_rad", 0.8);
         // 读取参数：发布频率
        this->declare_parameter("sensing.rate", 1.0);
        // 获取参数值
        _init_x = this->get_parameter("init_state_x").as_double();
        _init_y = this->get_parameter("init_state_y").as_double();
        
        _x_size = this->get_parameter("map.x_size").as_double();
        _y_size = this->get_parameter("map.y_size").as_double();
        _z_size = this->get_parameter("map.z_size").as_double();
        _obs_num = this->get_parameter("map.obs_num").as_int();
        _cir_num = this->get_parameter("map.circle_num").as_int();
        _resolution = this->get_parameter("map.resolution").as_double();
        
        _generate_ground = this->get_parameter("map.generate_ground").as_int();
        _max_ground_slope = this->get_parameter("map.max_ground_slope").as_double();
        
        _w_l = this->get_parameter("ObstacleShape.lower_rad").as_double();
        _w_h = this->get_parameter("ObstacleShape.upper_rad").as_double();
        _h_l = this->get_parameter("ObstacleShape.lower_hei").as_double();
        _h_h = this->get_parameter("ObstacleShape.upper_hei").as_double();
        
        _w_c_l = this->get_parameter("CircleShape.lower_circle_rad").as_double();
        _w_c_h = this->get_parameter("CircleShape.upper_circle_rad").as_double();
        
        _sense_rate = this->get_parameter("sensing.rate").as_double();
        // 计算地图边界
        _x_l = -_x_size / 2.0;
        _x_h = +_x_size / 2.0;
        _y_l = -_y_size / 2.0;
        _y_h = +_y_size / 2.0;

         // 创建发布器
        _all_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("global_map", 10);

        // 生成随机地图
        RandomMapGenerate();

        // 创建定时器，定期发布地图
        auto timer_period = std::chrono::duration<double>(1.0 / _sense_rate);
        _timer = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
            std::bind(&RandomComplexGenerator::pubSensedPoints, this));

        RCLCPP_INFO(this->get_logger(), "Random Complex Generator Node Started!");
        RCLCPP_INFO(this->get_logger(), "Map size: [%.1f, %.1f, %.1f]", _x_size, _y_size, _z_size);
        RCLCPP_INFO(this->get_logger(), "Obstacles: %d, Circles: %d", _obs_num, _cir_num);
    }
private:
    /**
     * @brief 计算给定位置的地面高度
     * @param x X坐标
     * @param y Y坐标
     * @return 地面高度（如果未生成地面则返回0）
     */
    double getGroundHeight(double x, double y)
    {
        if(_generate_ground != 1)
            return 0.0;
        
        double z_ground = _ground_slope_x * x + _ground_slope_y * y;
        return (z_ground < 0.0) ? 0.0 : z_ground;
    }

    /**
     * @brief 随机生成复杂场景地图
     */
    void RandomMapGenerate()
    {  
        // 初始化随机数生成器
        random_device rd;
        default_random_engine eng(rd());
        
        pcl::PointXYZ pt_random;
        
        // ============ 生成地面 ============
        if(_generate_ground == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Generating ground with max slope: %.2f", _max_ground_slope);
            
            // 定义地面坡度随机分布
            uniform_real_distribution<double> rand_slope = uniform_real_distribution<double>(-_max_ground_slope, _max_ground_slope);
            
            // 随机生成地面的坡度参数 (平面方程: z = ax + by + c)
            double slope_x = rand_slope(eng);  // x方向坡度
            double slope_y = rand_slope(eng);  // y方向坡度
            
            // 保存地面坡度参数供障碍物生成使用
            _ground_slope_x = slope_x;
            _ground_slope_y = slope_y;
            
            // 生成地面点云（包括地面以下的填充）
            for(double x = _x_l; x <= _x_h; x += _resolution)
            {
                for(double y = _y_l; y <= _y_h; y += _resolution)
                {
                    // 计算该点的地面高度（带坡度）
                    double z_ground = slope_x * x + slope_y * y;
                    
                    // 确保地面高度不为负
                    if(z_ground < 0.0)
                        z_ground = 0.0;
                    
                    // 填充地面以下的空间，从z=0到地面高度
                    // 使用更密集的填充确保没有空隙
                    for(double z = 0.0; z <= z_ground + _resolution; z += _resolution)
                    {
                        pt_random.x = x + 0.001;
                        pt_random.y = y + 0.001;
                        pt_random.z = z + 0.001;
                        
                        cloudMap.points.push_back(pt_random);
                    }
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Ground generated with slopes: x=%.3f, y=%.3f (including underground fill)", slope_x, slope_y);
        }
        
        // 定义柱状障碍物的随机分布范围
        uniform_real_distribution<double> rand_x = uniform_real_distribution<double>(_x_l, _x_h);
        uniform_real_distribution<double> rand_y = uniform_real_distribution<double>(_y_l, _y_h);
        uniform_real_distribution<double> rand_w = uniform_real_distribution<double>(_w_l, _w_h);
        uniform_real_distribution<double> rand_h = uniform_real_distribution<double>(_h_l, _h_h);

        // 定义圆形障碍物的随机分布范围
        uniform_real_distribution<double> rand_x_circle = uniform_real_distribution<double>(_x_l + 1.0, _x_h - 1.0);
        uniform_real_distribution<double> rand_y_circle = uniform_real_distribution<double>(_y_l + 1.0, _y_h - 1.0);
        uniform_real_distribution<double> rand_r_circle = uniform_real_distribution<double>(_w_c_l, _w_c_h);

        // 定义随机旋转角度分布
        uniform_real_distribution<double> rand_roll = uniform_real_distribution<double>(-M_PI, +M_PI);
        uniform_real_distribution<double> rand_pitch = uniform_real_distribution<double>(+M_PI/4.0, +M_PI/2.0);
        uniform_real_distribution<double> rand_yaw = uniform_real_distribution<double>(+M_PI/4.0, +M_PI/2.0);
        
        // 定义椭圆参数随机分布
        uniform_real_distribution<double> rand_ellipse_c = uniform_real_distribution<double>(0.5, 2.0);
        uniform_real_distribution<double> rand_num = uniform_real_distribution<double>(0.0, 1.0);

        // ============ 生成圆形/椭圆形障碍物 ============
        for(int i = 0; i < _cir_num; i++)
        {

            double x0, y0, z0, R;
            std::vector<Vector3d> circle_set;
            // 随机生成圆心和半径
            x0 = rand_x_circle(eng);
            y0 = rand_y_circle(eng);
            z0 = rand_h(eng) / 2.0;
            R = rand_r_circle(eng);

            //圆心距离初始位置过近则重新生成
            if(sqrt(pow(x0 - _init_x, 2) + pow(y0 - _init_y, 2)) < 2.0) 
                continue;
            
            //生成椭圆系数
            double a, b;
            a = rand_ellipse_c(eng);
            b = rand_ellipse_c(eng);

            // 生成圆/椭圆点集
            double x, y, z;
            Vector3d pt3, pt3_rot;
            for(double theta = -M_PI; theta < M_PI; theta += 0.025)
            {  
                x = a * cos(theta) * R;
                y = b * sin(theta) * R;
                z = 0;
                pt3 << x, y, z;
                circle_set.push_back(pt3);
            }

            // 定义随机3D旋转矩阵
            Matrix3d Rot;
            double roll, pitch, yaw;
            double alpha, beta, gama;
            
            roll = rand_roll(eng);
            pitch = rand_pitch(eng);
            yaw = rand_yaw(eng);

            alpha = roll;
            beta = pitch;
            gama = yaw;


            //50%概率绕某两个轴旋转90度，生成直立的椭圆
            double p = rand_num(eng);
            if(p < 0.5)
            {
                beta = M_PI / 2.0;
                gama = M_PI / 2.0;
            }

            // 计算旋转矩阵
            Rot << cos(alpha) * cos(gama) - cos(beta) * sin(alpha) * sin(gama), 
                   -cos(beta) * cos(gama) * sin(alpha) - cos(alpha) * sin(gama),   
                   sin(alpha) * sin(beta),
                   
                   cos(gama) * sin(alpha) + cos(alpha) * cos(beta) * sin(gama),   
                   cos(alpha) * cos(beta) * cos(gama) - sin(alpha) * sin(gama), 
                   -cos(alpha) * sin(beta),        
                   
                   sin(beta) * sin(gama),                                         
                   cos(gama) * sin(beta),                                         
                   cos(beta);

            // 将旋转后的点加入点云
            for(auto pt: circle_set)
            {
                pt3_rot = Rot * pt;
                pt_random.x = pt3_rot(0) + x0 + 0.001;
                pt_random.y = pt3_rot(1) + y0 + 0.001;
                pt_random.z = pt3_rot(2) + z0 + 0.001;

                // 计算该位置的地面高度
                double ground_height = getGroundHeight(pt_random.x, pt_random.y);
                
                // 只添加高于地面的点
                if(pt_random.z >= ground_height)
                    cloudMap.points.push_back(pt_random);
            }
        }
        // 构建KD树
        bool is_kdtree_empty = false;
        if(cloudMap.points.size() > 0)
            kdtreeMap.setInputCloud(cloudMap.makeShared()); 
        else
            is_kdtree_empty = true;

        // ============ 生成柱状障碍物 ============
        for(int i = 0; i < _obs_num; i++)
        {
            // 随机生成柱状障碍物的中心位置、半径和高度
            double x, y, w, h; 
            x = rand_x(eng);
            y = rand_y(eng);
            w = rand_w(eng);

            //距离初始位置过近则重新生成
            if(sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 0.8) 
                continue;

            // 检查新障碍物是否与已有障碍物冲突
            pcl::PointXYZ searchPoint(x, y, (_h_l + _h_h)/2.0);
            pointIdxSearch.clear();
            pointSquaredDistance.clear();

            //距离已有点过近则重新生成
            if(is_kdtree_empty == false)
            {
                if(kdtreeMap.nearestKSearch(searchPoint, 1, pointIdxSearch, pointSquaredDistance) > 0)
                {
                    if(sqrt(pointSquaredDistance[0]) < 1.0)
                        continue;
                }
            }

            // 将柱状障碍物点加入点云
            // 将中心点对齐到栅格中心
            x = floor(x / _resolution) * _resolution + _resolution / 2.0;
            y = floor(y / _resolution) * _resolution + _resolution / 2.0;

            // 计算柱状障碍物在栅格中的占用范围
            int widNum = ceil(w / _resolution);
            
            // 在占用范围内填充点云
            for(int r = -widNum/2.0; r < widNum/2.0; r++)
            {
                for(int s = -widNum/2.0; s < widNum/2.0; s++)
                {
                    h = rand_h(eng);
                    int heiNum = 2.0 * ceil(h / _resolution);
                    
                    // 计算该位置的地面高度
                    double pos_x = x + (r + 0.0) * _resolution + 0.001;
                    double pos_y = y + (s + 0.0) * _resolution + 0.001;
                    double ground_height = getGroundHeight(pos_x, pos_y);
                    
                    for(int t = 0; t < heiNum; t++)
                    {
                        pt_random.x = pos_x;
                        pt_random.y = pos_y;
                        pt_random.z = ground_height + (t + 0.0) * _resolution * 0.5 + 0.001;
                        cloudMap.points.push_back(pt_random);
                    }
                }
            }

        }
        // 将点云转换为ROS消息格式
        cloudMap.width = cloudMap.points.size();
        cloudMap.height = 1;
        cloudMap.is_dense = true;

        // 设置已有地图标志
        _has_map = true;
        
        // 转换为ROS消息格式
        pcl::toROSMsg(cloudMap, globalMap_pcd);
        globalMap_pcd.header.frame_id = "world";
        // 输出生成信息
        RCLCPP_INFO(this->get_logger(), "Map generated with %zu points", cloudMap.points.size());
    }

    /**
     * @brief 发布全局地图点云
     */
    void pubSensedPoints()
    {     
        if(!_has_map) return;

        globalMap_pcd.header.stamp = this->now();
        _all_map_pub->publish(globalMap_pcd);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _all_map_pub;
    rclcpp::TimerBase::SharedPtr _timer;

    // 地图参数
    int _obs_num, _cir_num;
    double _x_size, _y_size, _z_size;
    double _init_x, _init_y;
    double _resolution;
    double _sense_rate;
    double _x_l, _x_h;
    double _y_l, _y_h;
    double _w_l, _w_h;
    double _h_l, _h_h;
    double _w_c_l, _w_c_h;
    int _generate_ground;
    double _max_ground_slope;
    double _ground_slope_x, _ground_slope_y;  // 地面坡度参数
    bool _has_map = false;

    // 点云数据
    sensor_msgs::msg::PointCloud2 globalMap_pcd;
    pcl::PointCloud<pcl::PointXYZ> cloudMap;
    pcl::search::KdTree<pcl::PointXYZ> kdtreeMap;
    vector<int> pointIdxSearch;
    vector<float> pointSquaredDistance;
};

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RandomComplexGenerator>("random_complex_generator");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
};