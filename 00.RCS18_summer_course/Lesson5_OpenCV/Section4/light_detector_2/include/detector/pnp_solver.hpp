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
    PnpSolver();

    bool initCamera(const std::string &camera_config_path);

    bool solve(const std::vector<cv::Point3f> &object_points,
               const std::vector<cv::Point2f> &image_points,
               cv::Mat &rvec_out,
               cv::Mat &tvec_out);

private:
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    bool loadCameraParameters(const std::string &path);
};
#endif // PNP_SOLVER_HPP