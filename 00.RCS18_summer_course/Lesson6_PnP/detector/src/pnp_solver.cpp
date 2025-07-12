// pnp_solver.cpp
#include "pnp_solver.hpp"
#include "armor.hpp" // 需要包含 armor.hpp 来使用 Armor 和 Armor_params 结构体

#include <iostream>
#include <algorithm>                    // 包含 std::sort
#include <opencv2/calib3d.hpp>          // 包含 cv::solvePnP
#include <opencv2/core/persistence.hpp> // 包含 cv::FileStorage
#include <opencv2/imgproc.hpp>          // 包含 cv::minAreaRect

PnpSolver::PnpSolver()
{
    std::cout << "PnpSolver: Initialized." << std::endl;
}

// 1. 初始化相机内参和畸变系数
bool PnpSolver::initCamera(const std::string &camera_config_path)
{
    return loadCameraParameters(camera_config_path);
}

bool PnpSolver::loadCameraParameters(const std::string &path)
{
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "ERROR: PnpSolver: Could not open camera config file: " << path << std::endl;
        return false;
    }
    fs["camera_matrix"] >> camera_matrix_;
    fs["distortion_coefficients"] >> dist_coeffs_;

    if (camera_matrix_.empty() || dist_coeffs_.empty())
    {
        std::cerr << "ERROR: PnpSolver: Failed to read camera_matrix or distortion_coefficients from " << path << std::endl;
        return false;
    }
    std::cout << "PnpSolver: Camera parameters loaded successfully from " << path << std::endl;
    return true;
}

// 辅助函数：将四个顶点按 左上、右上、右下、左下 的顺序排序
// (PnP要求点顺序与3D模型对应)
void PnpSolver::sortPointsForPnP(std::vector<cv::Point2f> &points)
{
    // Sort by y-coordinate primarily, then by x-coordinate
    std::sort(points.begin(), points.end(), [](const cv::Point2f &a, const cv::Point2f &b)
              {
        // If y-coordinates are very close, sort by x to handle horizontal lines
        if (std::abs(a.y - b.y) < 5) {
            return a.x < b.x;
        }
        return a.y < b.y; });

    // After initial sort, we expect points to be somewhat ordered by y.
    // Let's refine to ensure TL, TR, BR, BL order.
    // Assuming points[0] and points[1] are top points, points[2] and points[3] are bottom points.
    // Sort top two points by x
    if (points[0].x > points[1].x)
        std::swap(points[0], points[1]);
    // Sort bottom two points by x
    if (points[2].x > points[3].x)
        std::swap(points[2], points[3]);

    // Reconstruct the order: TL, TR, BR, BL
    // Currently: points[0]=TL, points[1]=TR, points[2]=BL, points[3]=BR
    std::vector<cv::Point2f> sorted_output(4);
    sorted_output[0] = points[0]; // Left-Top
    sorted_output[1] = points[1]; // Right-Top
    sorted_output[2] = points[3]; // Right-Bottom (originally points[3] after x-sort)
    sorted_output[3] = points[2]; // Left-Bottom (originally points[2] after x-sort)

    points = sorted_output;
}

// 2. 从原始轮廓中提取装甲板的2D像素角点
std::vector<cv::Point2f> PnpSolver::obtain2DCorners(const std::vector<std::vector<cv::Point>> &all_raw_contours)
{
    std::vector<cv::Point2f> image_points; // 最终的2D角点

    // 1. 筛选出有效的灯条轮廓
    std::vector<cv::RotatedRect> light_bar_rects;
    for (const auto &contour : all_raw_contours)
    {
        if (contour.empty() || contour.size() < 5)
            continue; // 至少需要5个点来拟合旋转矩形

        cv::RotatedRect rect = cv::minAreaRect(contour);
        double area = rect.size.area();
        double aspect_ratio = rect.size.width / rect.size.height;
        if (aspect_ratio < 1)
            aspect_ratio = 1 / aspect_ratio; // 保持长宽比 >= 1 (垂直灯条)

        // 示例阈值 - 你需要根据你的实际灯条和图像来调整
        // 假设灯条通常是细长的，并且有一定面积
        if (area > 100 && area < 10000 && aspect_ratio > 2.0 && aspect_ratio < 10.0)
        {
            light_bar_rects.push_back(rect);
        }
    }

    if (light_bar_rects.size() < 2)
    {
        std::cout << "PnpSolver::obtain2DCorners: Not enough valid light bars found (need at least 2)." << std::endl;
        return {}; // 返回空vector表示失败
    }

    // 2. 灯条配对：找到最有可能形成装甲板的两个灯条
    // 在只保留灯条的简化场景下，我们假设最大的两个就是装甲板的灯条
    if (light_bar_rects.size() > 2)
    {
        std::sort(light_bar_rects.begin(), light_bar_rects.end(),
                  [](const cv::RotatedRect &a, const cv::RotatedRect &b)
                  {
                      return a.size.area() > b.size.area();
                  });
        light_bar_rects.resize(2); // 只保留最大的两个
    }

    if (light_bar_rects.size() != 2)
    {
        return {}; // 无法形成有效装甲板
    }

    // 3. 提取装甲板的四个外角点
    std::vector<cv::Point2f> all_light_points;
    cv::Point2f pts[4];

    // 获取第一个灯条的四个顶点
    light_bar_rects[0].points(pts);
    for (int i = 0; i < 4; ++i)
        all_light_points.push_back(pts[i]);

    // 获取第二个灯条的四个顶点
    light_bar_rects[1].points(pts);
    for (int i = 0; i < 4; ++i)
        all_light_points.push_back(pts[i]);

    if (all_light_points.empty())
    {
        std::cerr << "ERROR: PnpSolver::obtain2DCorners: No light points gathered after pairing." << std::endl;
        return {};
    }

    // 计算包含所有灯条顶点的最小外接矩形，作为装甲板的近似外形
    cv::RotatedRect armor_overall_rect = cv::minAreaRect(all_light_points);
    cv::Point2f armor_corners[4];
    armor_overall_rect.points(armor_corners);

    // 将四个角点复制到输出向量
    image_points.assign(armor_corners, armor_corners + 4);

    // 4. 关键步骤：对这些角点进行排序，使其顺序与 PnP 的 3D 世界点一致 (左上、右上、右下、左下)
    if (image_points.size() == 4)
    {
        sortPointsForPnP(image_points);
    }
    else
    {
        std::cerr << "PnpSolver::obtain2DCorners: Failed to obtain 4 sorted armor corners." << std::endl;
        return {};
    }

    return image_points;
}

// 3. 根据装甲板物理参数获取3D世界坐标点
std::vector<cv::Point3f> PnpSolver::obtain3DCorners(const Armor_params &armor_physical_params)
{
    // Armor_params 的构造函数已经根据装甲板尺寸计算好了 object_points
    // 所以这里直接返回它即可
    return armor_physical_params.object_points;
}

// 4. 执行 PnP 解算
bool PnpSolver::solvePnP(const std::vector<cv::Point3f> &object_points,
                         const std::vector<cv::Point2f> &image_points,
                         cv::Mat &rvec_out,
                         cv::Mat &tvec_out)
{
    // 防爆措施：检查输入点的数量
    if (image_points.size() != object_points.size() || image_points.size() < 4)
    {
        std::cerr << "ERROR: PnpSolver::solvePnP: Not enough corresponding points for PnP (need at least 4 pairs)." << std::endl;
        return false;
    }
    // 防爆措施：检查相机参数是否已加载
    if (camera_matrix_.empty() || dist_coeffs_.empty())
    {
        std::cerr << "ERROR: PnpSolver::solvePnP: Camera parameters not initialized. Call initCamera() first." << std::endl;
        return false;
    }

    // 调用 OpenCV 的 solvePnP 函数
    bool success = cv::solvePnP(object_points, image_points, camera_matrix_, dist_coeffs_,
                                rvec_out, tvec_out,
                                false,                   // useExtrinsicGuess = false，不使用外部初始猜测
                                cv::SOLVEPNP_ITERATIVE); // 可以尝试其他算法如 SOLVEPNP_EPNP, SOLVEPNP_SQPNP

    if (!success)
    {
        std::cerr << "WARNING: PnpSolver::solvePnP: failed. Check point correspondence and camera parameters." << std::endl;
    }
    else
    {
        std::cout << "PnpSolver::solvePnP: solution successful." << std::endl;
    }
    return success;
}