#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>
#include <string>

using namespace std::chrono_literals;

class ParamsVisualizer : public rclcpp::Node {
public:
    ParamsVisualizer() : Node("params_visualizer") {
        // 创建MarkerArray发布者
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "visualization_marker_array", 10);
        
        // 声明参数
        this->declare_parameter("special_area.upanddown_path", std::vector<std::string>());
        this->declare_parameter("special_area.upanddown_polygon", std::vector<std::string>());
        this->declare_parameter("special_area.move_polygon", std::vector<std::string>());
        this->declare_parameter("red_goals_array.cruise_goals", std::vector<std::string>());
        this->declare_parameter("blue_goals_array.cruise_goals", std::vector<std::string>());
        
        // 立即尝试发布一次（等待参数加载）
        rclcpp::sleep_for(500ms);
        
        // 读取参数
        if (!load_params()) {
            RCLCPP_ERROR(this->get_logger(), "参数加载失败，使用默认硬编码值");
            use_default_values();
        }
        
        // 创建定时器，定期发布可视化标记
        timer_ = this->create_wall_timer(1s, [this]() {
            publish_visualization();
        });
        
        RCLCPP_INFO(this->get_logger(), "Params Visualizer started.");
        RCLCPP_INFO(this->get_logger(), "请启动RViz并添加MarkerArray显示（Topic: /visualization_marker_array）");
    }

private:
    // 从参数字符串解析点坐标
    std::vector<std::pair<double, double>> parse_points(const std::vector<std::string>& str_points) {
        std::vector<std::pair<double, double>> result;
        for (const auto& s : str_points) {
            size_t comma = s.find(',');
            if (comma != std::string::npos) {
                double x = std::stod(s.substr(0, comma));
                double y = std::stod(s.substr(comma + 1));
                result.push_back({x, y});
            }
        }
        return result;
    }
    
    bool load_params() {
        auto upanddown_path_str = this->get_parameter_or("special_area.upanddown_path", std::vector<std::string>());
        auto upanddown_polygon_str = this->get_parameter_or("special_area.upanddown_polygon", std::vector<std::string>());
        auto move_polygon_str = this->get_parameter_or("special_area.move_polygon", std::vector<std::string>());
        auto cruise_goals_str = this->get_parameter_or("red_goals_array.cruise_goals", std::vector<std::string>());
        
        if (upanddown_path_str.empty() || upanddown_polygon_str.empty() || 
            move_polygon_str.empty() || cruise_goals_str.empty()) {
            RCLCPP_WARN(this->get_logger(), "部分参数为空，请检查参数文件");
            return false;
        }
        
        upanddown_path_ = parse_points(upanddown_path_str);
        upanddown_polygon_ = parse_points(upanddown_polygon_str);
        move_polygon_ = parse_points(move_polygon_str);
        cruise_goals_ = parse_points(cruise_goals_str);
        
        RCLCPP_INFO(this->get_logger(), "参数加载成功:");
        RCLCPP_INFO(this->get_logger(), "  upanddown_path: %zu points", upanddown_path_.size());
        RCLCPP_INFO(this->get_logger(), "  upanddown_polygon: %zu points", upanddown_polygon_.size());
        RCLCPP_INFO(this->get_logger(), "  move_polygon: %zu points", move_polygon_.size());
        RCLCPP_INFO(this->get_logger(), "  cruise_goals: %zu points", cruise_goals_.size());
        
        return true;
    }
    
    void use_default_values() {
        // 使用默认硬编码值
        upanddown_path_ = {{3.04, -2.08}, {4.46, 0.04}, {6.20, 1.25}, {7.65, 3.58}};
        upanddown_polygon_ = {{6.55, 2.31}, {7.37, 1.19}, {4.09, -1.09}, {3.16, 0.04}};
        move_polygon_ = {{10.06, 0.97}, {7.03, 5.23}, {0.96, 1.03}, {0.51, -1.44}, {2.71, -4.22}};
        cruise_goals_ = {{1.98, -0.90}, {8.79, 2.19}};
    }

private:
    void publish_visualization() {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        
        // ===== 1. 起伏路段区域 (upanddown_polygon) - 蓝色填充 =====
        auto upanddown_line = create_polygon_marker(upanddown_polygon_, id++, "upanddown_polygon", 
                                                     0.0, 0.0, 1.0, 0.1);
        marker_array.markers.push_back(upanddown_line);
        
        auto upanddown_fill = create_filled_polygon_marker(upanddown_polygon_, id++, "upanddown_fill",
                                                           0.0, 0.0, 1.0, 0.3);
        marker_array.markers.push_back(upanddown_fill);
        
        auto upanddown_points = create_points_marker(upanddown_polygon_, id++, "upanddown_points",
                                                      0.0, 0.0, 1.0);
        marker_array.markers.push_back(upanddown_points);
        
        auto upanddown_text = create_text_marker(4.5, 0.5, 0.5, "UpAndDown Area", id++, "upanddown_text");
        marker_array.markers.push_back(upanddown_text);
        
        // ===== 2. 移动区域 (move_polygon) - 蓝色填充 =====
        auto move_line = create_polygon_marker(move_polygon_, id++, "move_polygon",
                                               0.0, 0.0, 1.0, 0.05);
        marker_array.markers.push_back(move_line);
        
        auto move_fill = create_filled_polygon_marker(move_polygon_, id++, "move_fill",
                                                      0.0, 0.0, 1.0, 0.15);
        marker_array.markers.push_back(move_fill);
        
        auto move_points = create_points_marker(move_polygon_, id++, "move_points",
                                                0.0, 0.0, 1.0);
        marker_array.markers.push_back(move_points);
        
        auto move_text = create_text_marker(5.0, 0.0, 0.5, "Move Area", id++, "move_text");
        marker_array.markers.push_back(move_text);
        
        // ===== 3. 起伏路段路径 (upanddown_path) - 绿色 =====
        auto path_line = create_path_marker(upanddown_path_, id++, "upanddown_path",
                                            0.0, 1.0, 0.0, 0.1);
        marker_array.markers.push_back(path_line);
        
        auto path_points = create_points_marker(upanddown_path_, id++, "upanddown_path_points",
                                                0.0, 1.0, 0.0);
        marker_array.markers.push_back(path_points);
        
        std::vector<std::string> path_labels = {"Start", "Begin", "End", "Exit"};
        for (size_t i = 0; i < upanddown_path_.size() && i < path_labels.size(); ++i) {
            auto label = create_text_marker(upanddown_path_[i].first, upanddown_path_[i].second, 0.3, 
                                           path_labels[i], id++, "path_label_" + std::to_string(i));
            marker_array.markers.push_back(label);
        }
        
        // ===== 4. 巡航目标点 (cruise_goals) - 绿色 =====
        auto cruise_points = create_points_marker(cruise_goals_, id++, "cruise_goals",
                                                  0.0, 1.0, 0.0, 0.3);
        marker_array.markers.push_back(cruise_points);
        
        for (size_t i = 0; i < cruise_goals_.size(); ++i) {
            std::string label = "Goal " + std::to_string(i + 1);
            auto text = create_text_marker(cruise_goals_[i].first, cruise_goals_[i].second, 0.5,
                                          label, id++, "cruise_label_" + std::to_string(i));
            marker_array.markers.push_back(text);
        }
        
        // 发布所有标记
        marker_pub_->publish(marker_array);
        
        static bool first_publish = true;
        if (first_publish) {
            RCLCPP_INFO(this->get_logger(), "已发布 %zu 个可视化标记", marker_array.markers.size());
            first_publish = false;
        }
    }
    
    // 创建多边形边界线标记
    visualization_msgs::msg::Marker create_polygon_marker(
        const std::vector<std::pair<double, double>>& points,
        int id, const std::string& ns,
        float r, float g, float b, float line_width) {
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        marker.scale.x = line_width;
        
        for (const auto& p : points) {
            geometry_msgs::msg::Point pt;
            pt.x = p.first;
            pt.y = p.second;
            pt.z = 0.0;
            marker.points.push_back(pt);
        }
        // 闭合多边形
        if (!points.empty()) {
            geometry_msgs::msg::Point pt;
            pt.x = points.front().first;
            pt.y = points.front().second;
            pt.z = 0.0;
            marker.points.push_back(pt);
        }
        
        return marker;
    }
    
    // 创建填充多边形标记（使用TRIANGLE_LIST）
    visualization_msgs::msg::Marker create_filled_polygon_marker(
        const std::vector<std::pair<double, double>>& points,
        int id, const std::string& ns,
        float r, float g, float b, float alpha) {
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = alpha;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        
        // 使用扇形三角化填充多边形（从第一个点出发）
        if (points.size() >= 3) {
            geometry_msgs::msg::Point center;
            center.x = 0;
            center.y = 0;
            center.z = 0;
            for (const auto& p : points) {
                center.x += p.first;
                center.y += p.second;
            }
            center.x /= points.size();
            center.y /= points.size();
            
            for (size_t i = 0; i < points.size(); ++i) {
                size_t next = (i + 1) % points.size();
                
                geometry_msgs::msg::Point pt1;
                pt1.x = center.x;
                pt1.y = center.y;
                pt1.z = 0.0;
                
                geometry_msgs::msg::Point pt2;
                pt2.x = points[i].first;
                pt2.y = points[i].second;
                pt2.z = 0.0;
                
                geometry_msgs::msg::Point pt3;
                pt3.x = points[next].first;
                pt3.y = points[next].second;
                pt3.z = 0.0;
                
                marker.points.push_back(pt1);
                marker.points.push_back(pt2);
                marker.points.push_back(pt3);
            }
        }
        
        return marker;
    }
    
    // 创建路径线标记
    visualization_msgs::msg::Marker create_path_marker(
        const std::vector<std::pair<double, double>>& points,
        int id, const std::string& ns,
        float r, float g, float b, float line_width) {
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        marker.scale.x = line_width;
        
        for (const auto& p : points) {
            geometry_msgs::msg::Point pt;
            pt.x = p.first;
            pt.y = p.second;
            pt.z = 0.05;  // 稍微抬高一点，避免与区域重叠
            marker.points.push_back(pt);
        }
        
        return marker;
    }
    
    // 创建点标记
    visualization_msgs::msg::Marker create_points_marker(
        const std::vector<std::pair<double, double>>& points,
        int id, const std::string& ns,
        float r, float g, float b, float size = 0.15) {
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;
        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        
        for (const auto& p : points) {
            geometry_msgs::msg::Point pt;
            pt.x = p.first;
            pt.y = p.second;
            pt.z = 0.1;
            marker.points.push_back(pt);
        }
        
        return marker;
    }
    
    // 创建文字标记
    visualization_msgs::msg::Marker create_text_marker(
        double x, double y, double z, const std::string& text,
        int id, const std::string& ns) {
        
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.scale.z = 0.25;  // 文字大小
        
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        
        marker.text = text;
        
        return marker;
    }
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 存储从参数读取的点坐标
    std::vector<std::pair<double, double>> upanddown_path_;
    std::vector<std::pair<double, double>> upanddown_polygon_;
    std::vector<std::pair<double, double>> move_polygon_;
    std::vector<std::pair<double, double>> cruise_goals_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParamsVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
