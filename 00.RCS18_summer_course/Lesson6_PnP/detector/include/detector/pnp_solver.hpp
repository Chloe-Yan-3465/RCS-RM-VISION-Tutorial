#ifndef PNP_SOLVER_HPP
#define PNP_SOLVER_HPP

#include <string>
#include <vector>
#include <opencv2/core.hpp>

// 前向声明 Armor 结构体，避免交叉包含和编译依赖，只在这里声明
struct Armor;
struct Armor_params;

class PnpSolver
{
public:
    PnpSolver(); // 构造函数

    // 1. 初始化相机内参和畸变系数
    bool initCamera(const std::string &camera_config_path);

    // 2. 从原始轮廓中提取装甲板的2D像素角点
    // 包含灯条筛选、配对、minAreaRect、points()、以及角点排序
    // 返回空vector表示失败
    std::vector<cv::Point2f> obtain2DCorners(const std::vector<std::vector<cv::Point>> &all_raw_contours);

    // 3. 根据装甲板物理参数获取3D世界坐标点
    // 如果你在 Armor_params 构造函数中已经计算好了，这个函数只是返回它
    // 这样做是为了满足你“return一个vector”的要求
    std::vector<cv::Point3f> obtain3DCorners(const Armor_params &armor_physical_params);

    // 4. 执行 PnP 解算
    // 传入2D和3D点，输出rvec和tvec
    // 包含防爆措施（点数量检查、相机参数检查）
    bool solvePnP(const std::vector<cv::Point3f> &object_points,
                  const std::vector<cv::Point2f> &image_points,
                  cv::Mat &rvec_out,
                  cv::Mat &tvec_out);

private:
    cv::Mat camera_matrix_; // 相机内参矩阵
    cv::Mat dist_coeffs_;   // 畸变系数

    // 私有辅助函数：加载相机参数
    bool loadCameraParameters(const std::string &path);

    // 私有辅助函数：将四个顶点按 左上、右上、右下、左下 的顺序排序
    // (PnP要求点顺序与3D模型对应)
    static void sortPointsForPnP(std::vector<cv::Point2f> &points);
};
#endif // PNP_SOLVER_HPP