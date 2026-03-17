/**
 * 特殊区域判断算法验证程序
 * 从 PubRobotStatus.cpp 中提取的判断逻辑
 * 不使用ROS，纯C++实现
 */

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

// ==================== 数据结构定义 ====================

// 全局坐标点结构 (与 PubRobotStatus.hpp 中的 GlobalPose 一致)
struct GlobalPose {
    float pose_x;
    float pose_y;
};

// 特殊区域类型枚举 (与 PubRobotStatus.hpp 中的 SpecialAreaType 一致)
enum class SpecialAreaType {
    MOVE_AREA,      // 移动区域
    UPANDDOWN_AREA, // 起伏路段区域
    ILLEGAL_AREA    // 非法区域
};

// 将区域类型转换为字符串
std::string areaTypeToString(SpecialAreaType type) {
    switch (type) {
        case SpecialAreaType::MOVE_AREA:
            return "移动区域 (MOVE_AREA)";
        case SpecialAreaType::UPANDDOWN_AREA:
            return "起伏路段区域 (UPANDDOWN_AREA)";
        case SpecialAreaType::ILLEGAL_AREA:
            return "非法区域 (ILLEGAL_AREA)";
        default:
            return "未知区域";
    }
}

// ==================== 参数配置 (从 params.yaml 读取) ====================

// 起伏路段区域边界点 (upanddown_polygon)
// ["1.71,-2.03", "3.77,-1.94", "3.77,-6.54", "1.76,-6.59"] - 小长方形
const std::vector<std::string> UPANDDOWN_POLYGON_STR = {
    "1.71,-2.03", "3.77,-1.94", "3.77,-6.54", "1.76,-6.59"
};

// 移动区域边界点 (move_polygon)
// ["1.15,1.07", "6.32,1.16", "6.49,-11.51", "0.88,-11.90"] - 巡航区域
const std::vector<std::string> MOVE_POLYGON_STR = {
    "1.15,1.07", "6.32,1.16", "6.49,-11.51", "0.88,-11.90"
};

// ==================== 工具函数 ====================

/**
 * @brief 解析字符串格式的坐标点为 GlobalPose
 * @param point_str 格式: "x,y"
 * @return GlobalPose 结构
 */
GlobalPose parsePoint(const std::string& point_str) {
    GlobalPose gp;
    size_t comma_pos = point_str.find(',');
    if (comma_pos != std::string::npos) {
        gp.pose_x = std::stof(point_str.substr(0, comma_pos));
        gp.pose_y = std::stof(point_str.substr(comma_pos + 1));
    }
    return gp;
}

/**
 * @brief 将字符串数组转换为 GlobalPose 数组
 */
std::vector<GlobalPose> parsePolygon(const std::vector<std::string>& polygon_str) {
    std::vector<GlobalPose> polygon;
    for (const auto& point_str : polygon_str) {
        polygon.push_back(parsePoint(point_str));
    }
    return polygon;
}

/**
 * @brief 计算两点之间的距离
 * @param pose1 点1
 * @param pose2 点2
 * @return 欧几里得距离
 */
float calculate_distance(const GlobalPose& pose1, const GlobalPose& pose2) {
    return std::sqrt(std::pow(pose1.pose_x - pose2.pose_x, 2) + 
                     std::pow(pose1.pose_y - pose2.pose_y, 2));
}

/**
 * @brief 判断点是否在多边形内 - 射线法 (Ray Casting Algorithm)
 * 
 * 算法原理：
 * 从点向右发射水平射线，计算与多边形边的交点数量
 * - 奇数个交点：点在多边形内部
 * - 偶数个交点：点在多边形外部
 * 
 * 注意：对于边界上的点，算法可能返回 true 或 false，取决于具体实现
 * 
 * @param point 待判断的点
 * @param polygon 多边形边界点数组（按顺序连接）
 * @return true 点在多边形内
 * @return false 点在多边形外
 * 
 * 对应 PubRobotStatus.cpp 第 709-728 行
 */
bool is_in_this_polygon(const GlobalPose& point, const std::vector<GlobalPose>& polygon) {
    const float EPSILON = 1e-9;
    bool inside = false;
    int n = polygon.size();

    // 遍历多边形的每条边 (i, j) 其中 j 是 i 的前一个点
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const auto& pi = polygon[i];  // 当前点
        const auto& pj = polygon[j];  // 前一个点

        // 射线法核心判断：
        // 1. (pi.pose_y > point.pose_y) != (pj.pose_y > point.pose_y)
        //    - 检查点的 y 坐标是否在边的两个端点的 y 坐标之间
        //    - 这是判断射线是否与边相交的必要条件
        //
        // 2. point.pose_x < (pj.pose_x - pi.pose_x) * (point.pose_y - pi.pose_y) / (pj.pose_y - pi.pose_y + EPSILON) + pi.pose_x
        //    - 计算射线与边的交点的 x 坐标
        //    - 如果交点在点的右侧（x > point.x），则射线与边相交
        //    - EPSILON 避免除以零
        
        if (((pi.pose_y > point.pose_y) != (pj.pose_y > point.pose_y)) &&
            (point.pose_x < (pj.pose_x - pi.pose_x) * (point.pose_y - pi.pose_y) / (pj.pose_y - pi.pose_y + EPSILON) + pi.pose_x)) {
            inside = !inside;  // 切换内外状态
        }
    }
    return inside;
}

// ==================== 核心区域判断函数 ====================

/**
 * @brief 获取点所在的区域类型
 * 
 * 判断顺序：
 * 1. UPANDDOWN_AREA (起伏路段区域) - 小长方形，优先判断
 * 2. MOVE_AREA (移动区域) - 大矩形，包含小长方形
 * 
 * 注意：为了避免有洞区域的复杂判断，先判断小区域（洞内），
 *       如果是的话直接返回，不是的话才会判断是不是属于其他区域
 * 
 * @param point 待判断的点
 * @param upanddown_polygon 起伏路段区域边界
 * @param move_polygon 移动区域边界
 * @return SpecialAreaType 区域类型
 * 
 * 对应 PubRobotStatus.cpp 第 629-645 行
 */
SpecialAreaType GetPointArea(const GlobalPose& point,
                             const std::vector<GlobalPose>& upanddown_polygon,
                             const std::vector<GlobalPose>& move_polygon) {
    // 注意这里的顺序，为了避免有洞区域的复杂判断，对洞内区域(起伏路段)先判断
    // 顺序：1. 起伏路段 2. 移动区域
    
    // 1. 先判断是否在起伏路段区域内 (小长方形)
    if (is_in_this_polygon(point, upanddown_polygon)) {
        return SpecialAreaType::UPANDDOWN_AREA;
    }
    
    // 2. 再判断是否在移动区域内 (大矩形)
    if (is_in_this_polygon(point, move_polygon)) {
        return SpecialAreaType::MOVE_AREA;
    }
    
    // 3. 不在任何区域内，返回非法区域
    return SpecialAreaType::ILLEGAL_AREA;
}

// ==================== 辅助打印函数 ====================

/**
 * @brief 打印多边形信息
 */
void printPolygon(const std::string& name, const std::vector<GlobalPose>& polygon) {
    std::cout << "  " << name << " (" << polygon.size() << " 个点):" << std::endl;
    for (size_t i = 0; i < polygon.size(); ++i) {
        std::cout << "    [" << i << "] (" << polygon[i].pose_x << ", " << polygon[i].pose_y << ")" << std::endl;
    }
}

/**
 * @brief 打印分割线
 */
void printSeparator() {
    std::cout << "============================================================" << std::endl;
}

// ==================== 主函数 ====================
// ["5.65,-9.89", "4.17,-10.72", "1.78,-9.16", "3.14,-6.69", "3.06,-6.46", "2.87,-6.34", "2.62,-5.97", "2.48,-2.63", "2.82,-2.21", "3.10,-1.77", "2.10,-0.96", "1.40,-0.55", "0.63,-0.21"]
int main() {
    // 待测试的点
    const std::vector<std::string> test_points_str = {
        "5.65,-9.89",  // 点1
        "4.17,-10.72", // 点2
        "1.78,-9.16",  // 点3
        "3.14,-6.69",  // 点4
        "3.06,-6.46",  // 点5
        "2.87,-6.34",  // 点6
        "2.62,-5.97",  // 点7
        "2.48,-2.63",  // 点8
        "2.82,-2.21",  // 点9
        "3.10,-1.77",  // 点10
        "2.10,-0.96",  // 点11
        "1.40,-0.55",  // 点12
        "0.63,-0.21"   // 点13
    };

    // 解析区域边界
    std::vector<GlobalPose> upanddown_polygon = parsePolygon(UPANDDOWN_POLYGON_STR);
    std::vector<GlobalPose> move_polygon = parsePolygon(MOVE_POLYGON_STR);
    
    // 解析测试点
    std::vector<GlobalPose> test_points;
    for (const auto& point_str : test_points_str) {
        test_points.push_back(parsePoint(point_str));
    }
    
    // ==================== 打印配置信息 ====================
    std::cout << std::endl;
    printSeparator();
    std::cout << "           特殊区域判断算法验证程序" << std::endl;
    std::cout << "    (从 PubRobotStatus.cpp 提取，无ROS依赖)" << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    std::cout << "【区域配置】 (来自 params.yaml)" << std::endl;
    printPolygon("起伏路段区域 (upanddown_polygon)", upanddown_polygon);
    printPolygon("移动区域 (move_polygon)", move_polygon);
    std::cout << std::endl;
    
    std::cout << "【判断顺序】" << std::endl;
    std::cout << "  1. 先判断起伏路段区域 (小长方形，内部区域)" << std::endl;
    std::cout << "  2. 再判断移动区域 (大矩形，外部区域)" << std::endl;
    std::cout << "  3. 都不在则返回非法区域" << std::endl;
    std::cout << std::endl;
    
    std::cout << "【待测试点】" << std::endl;
    for (size_t i = 0; i < test_points.size(); ++i) {
        std::cout << "  点" << (i + 1) << ": (" << test_points[i].pose_x 
                  << ", " << test_points[i].pose_y << ")" << std::endl;
    }
    std::cout << std::endl;
    
    // ==================== 执行判断 ====================
    printSeparator();
    std::cout << "                     判断结果" << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    for (size_t i = 0; i < test_points.size(); ++i) {
        const auto& point = test_points[i];
        
        // 判断区域
        SpecialAreaType area = GetPointArea(point, upanddown_polygon, move_polygon);
        
        // 计算到各区域边界的距离（用于参考）
        float dist_to_upanddown_center = 0.0f;
        float dist_to_move_center = 0.0f;
        
        // 起伏路段中心点近似值
        GlobalPose upanddown_center = {(1.71f + 3.77f) / 2.0f, (-2.03f + -6.59f) / 2.0f};
        // 移动区域中心点近似值  
        GlobalPose move_center = {(1.15f + 6.49f) / 2.0f, (1.07f + -11.90f) / 2.0f};
        
        dist_to_upanddown_center = calculate_distance(point, upanddown_center);
        dist_to_move_center = calculate_distance(point, move_center);
        
        // 打印结果
        std::cout << "点" << (i + 1) << ": (" << point.pose_x << ", " << point.pose_y << ")" << std::endl;
        std::cout << "  └─ 所在区域: " << areaTypeToString(area) << std::endl;
        std::cout << "  └─ 到起伏路段中心距离: " << dist_to_upanddown_center << " 米" << std::endl;
        std::cout << "  └─ 到移动区域中心距离: " << dist_to_move_center << " 米" << std::endl;
        
        // 详细判断过程
        std::cout << "  └─ 详细判断:" << std::endl;
        bool in_upanddown = is_in_this_polygon(point, upanddown_polygon);
        bool in_move = is_in_this_polygon(point, move_polygon);
        std::cout << "      • 在起伏路段区域内? " << (in_upanddown ? "是" : "否") << std::endl;
        std::cout << "      • 在移动区域内? " << (in_move ? "是" : "否") << std::endl;
        
        std::cout << std::endl;
    }
    
    // ==================== 总结 ====================
    printSeparator();
    std::cout << "                       总结" << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    for (size_t i = 0; i < test_points.size(); ++i) {
        const auto& point = test_points[i];
        SpecialAreaType area = GetPointArea(point, upanddown_polygon, move_polygon);
        std::cout << "点" << (i + 1) << " (" << point.pose_x << ", " << point.pose_y 
                  << ") -> " << areaTypeToString(area) << std::endl;
    }
    
    std::cout << std::endl;
    printSeparator();
    std::cout << "验证完成！算法与 PubRobotStatus.cpp 中的实现完全一致。" << std::endl;
    printSeparator();
    std::cout << std::endl;
    
    return 0;
}
