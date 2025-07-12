#include "detector/detector.hpp"
#include "pnp_solver.hpp" // Include the PnpSolver header
#include "armor.hpp"      // Include the Armor and Armor_params header

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

// Assuming 'camera.yaml' exists in the build directory or specify its full path
// Example camera.yaml content (place this in your project root or build folder):
/*
%YAML:1.0
---
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 600.0, 0.0, 320.0,
           0.0, 600.0, 240.0,
           0.0, 0.0, 1.0 ]
distortion_coefficients: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ 0.1, -0.05, 0.001, 0.0005, 0.0 ]
*/

int main(int argc, char *argv[])
{
    //====== 1.处理命令行所传入的参数 ======
    // 检查参数数量：图像路径、阈值、检测颜色
    if (argc < 4)
    {
        std::cerr << "Usage: " << argv[0] << " <image_path> <binary_threshold> <detect_color (0:Red, 1:Blue)>" << std::endl;
        return -1;
    }

    // 读取图像路径、阈值和检测颜色参数
    std::string image_path = argv[1];
    int binary_thres = std::stoi(argv[2]); // 转换为 int
    int detect_color = std::stoi(argv[3]); // 检测颜色：0代表红色，1代表蓝色

    // 读取图像
    cv::Mat bgr_img = cv::imread(image_path, cv::IMREAD_COLOR);
    if (bgr_img.empty())
    {
        std::cerr << "ERROR: Could not read image from: " << image_path << std::endl;
        return -1;
    }
    std::cout << "1. Image loaded successfully." << std::endl;

    // ====== 2.初始化 Detector 和 PnpSolver 对象 ======
    Detector detector;    // 在栈上创建Detector对象
    PnpSolver pnp_solver; // 在栈上创建PnpSolver对象
    std::cout << "2. Detector and PnpSolver objects initialized." << std::endl;

    // ====== 3.检查图像颜色是否符合要求 ======
    if (!detector.checkColor(bgr_img, detect_color))
    {
        std::cout << "Invalid" << std::endl; // 不符合颜色要求，打印"Invalid"
        return 0;                            // 不符合则直接退出程序
    }
    std::cout << "3. Image color meets requirement, proceeding with processing." << std::endl;

    // ====== 4.预处理图像：灰度化 -> 二值化 ======
    cv::Mat binary_img = detector.preprocessImage(bgr_img, binary_thres);
    std::cout << "4. Image preprocessed (grayscale and binary)." << std::endl;

    // ====== 5.提取轮廓 ======
    std::vector<std::vector<cv::Point>> light_bar_contours = detector.findContoursInBinary(binary_img);
    std::cout << "5. Number of contours found: " << light_bar_contours.size() << std::endl;

    // ====== 6.绘制所有提取到的原始轮廓 ======
    // We'll draw on a clone of the original image to preserve it for later
    cv::Mat display_img = bgr_img.clone(); // Create a copy for drawing
    detector.drawContours(display_img, light_bar_contours);
    std::cout << "6. All raw contours drawn." << std::endl;

    // --- PnP Solver Integration ---

    // ====== 7. 初始化相机参数 (由 PnpSolver 负责加载) ======
    std::string camera_config_path = "camera.yaml"; // Make sure this file is accessible
    if (!pnp_solver.initCamera(camera_config_path))
    {
        std::cerr << "ERROR: Failed to initialize camera parameters. Exiting." << std::endl;
        return -1;
    }
    std::cout << "7. Camera parameters initialized by PnpSolver." << std::endl;

    // ====== 8. 定义装甲板的物理尺寸参数 ======
    // These dimensions should be in meters (m) and correspond to your physical armor plate
    // Example: small armor plate, width 13.5cm (0.135m), height 5.5cm (0.055m)
    // light_bar_width, light_bar_height, light_bar_spacing are parameters used internally by Armor_params
    Armor_params armor_params(0.135, 0.055, 0.02, 0.055, 0.1);
    std::cout << "8. Armor physical parameters (for 3D points) initialized." << std::endl;

    // ====== 9. 从原始轮廓中获取装甲板的四个 2D 像素角点 ======
    std::vector<cv::Point2f> image_points = pnp_solver.obtain2DCorners(light_bar_contours);
    bool corners_obtained = !image_points.empty();
    std::cout << "9. 2D armor corners obtained: " << (corners_obtained ? "Yes (" + std::to_string(image_points.size()) + " points)" : "No") << std::endl;

    // ====== 10. 获取装甲板的 3D 世界坐标点 ======
    // This calls obtain3DCorners which will return the object_points from your armor_params object
    std::vector<cv::Point3f> object_points = pnp_solver.obtain3DCorners(armor_params);
    // object_points should always have 4 points if Armor_params is correctly defined
    std::cout << "10. 3D armor object points obtained: " << (object_points.size() == 4 ? "Yes" : "No") << std::endl;

    // ====== 11. 执行 PnP 解算 ======
    cv::Mat rvec, tvec; // Declare rotation and translation vectors
    bool pnp_solved = false;
    if (corners_obtained && object_points.size() == 4)
    {
        pnp_solved = pnp_solver.solvePnP(object_points, image_points, rvec, tvec);
        std::cout << "11. PnP solution attempt completed. Success: " << (pnp_solved ? "Yes" : "No") << std::endl;
    }
    else
    {
        std::cout << "11. Skipping PnP solution due to insufficient 2D/3D points." << std::endl;
    }

    // ====== 12. 显示 PnP 结果并绘制装甲板姿态 ======
    if (pnp_solved)
    {
        std::cout << "   Rotation Vector (rvec): " << rvec.t() << std::endl;
        std::cout << "   Translation Vector (tvec): " << tvec.t() << std::endl;
        double distance = cv::norm(tvec); // Calculate distance from translation vector
        std::cout << "   Distance to armor: " << distance << " meters" << std::endl;

        // Draw the 4 PnP-solved corners on the image (on display_img)
        for (const auto &p : image_points)
        {
            cv::circle(display_img, p, 5, cv::Scalar(0, 255, 255), -1); // Yellow circles
        }
        // Draw lines connecting the corners to form the detected armor rectangle
        cv::line(display_img, image_points[0], image_points[1], cv::Scalar(0, 255, 255), 2);
        cv::line(display_img, image_points[1], image_points[2], cv::Scalar(0, 255, 255), 2);
        cv::line(display_img, image_points[2], image_points[3], cv::Scalar(0, 255, 255), 2);
        cv::line(display_img, image_points[3], image_points[0], cv::Scalar(0, 255, 255), 2);
        std::cout << "12. PnP corners and estimated pose displayed on image." << std::endl;
    }
    else
    {
        std::cout << "12. No PnP solution to display on image." << std::endl;
    }

    // ====== 13. 显示最终结果图像 ======
    std::cout << "13. Displaying results." << std::endl;
    cv::imshow("Detected Contours & Armor PnP", display_img);
    cv::waitKey(0); // Wait for a key press to close the window

    return 0;
}