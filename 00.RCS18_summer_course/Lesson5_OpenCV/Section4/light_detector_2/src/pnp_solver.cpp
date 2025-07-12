// pnp_solver.cpp
#include "detector/pnp_solver.hpp"
#include "armor.hpp" // 包含 armor.hpp 来使用 Armor 和 Armor_params 结构体

#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>

PnpSolver::PnpSolver()
{
    std::cout << "PnpSolver: Initialized." << std::endl;
}

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

bool PnpSolver::solve(const std::vector<cv::Point3f> &object_points,
                      const std::vector<cv::Point2f> &image_points,
                      cv::Mat &rvec_out,
                      cv::Mat &tvec_out)
{
    if (image_points.size() != object_points.size() || image_points.size() < 4)
    {
        std::cerr << "ERROR: PnpSolver: Not enough corresponding points for PnP (need at least 4 pairs)." << std::endl;
        return false;
    }
    if (camera_matrix_.empty() || dist_coeffs_.empty())
    {
        std::cerr << "ERROR: PnpSolver: Camera parameters not initialized. Call initCamera() first." << std::endl;
        return false;
    }

    bool success = cv::solvePnP(object_points, image_points, camera_matrix_, dist_coeffs_,
                                rvec_out, tvec_out,
                                false,
                                cv::SOLVEPNP_ITERATIVE);

    if (!success)
    {
        std::cerr << "WARNING: PnpSolver: solvePnP failed. Check point correspondence and camera parameters." << std::endl;
    }
    else
    {
        std::cout << "PnpSolver: PnP solution successful." << std::endl;
    }
    return success;
}