#include <opencv2/opencv.hpp> // 包含OpenCV库
#include <iostream>           // 包含输入输出流库
#include <string>             // 包含字符串库

int main(int argc, char *argv[])
{
    // 检查命令行参数数量
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <image_path> <binary_threshold>" << std::endl;
        return -1; // 参数不足，退出程序
    }

    // 从命令行参数中获取图像路径和二值化阈值
    std::string image_path = argv[1];
    int binary_threshold = std::stoi(argv[2]); // 将字符串阈值转换为整数

    // 读取彩色图像
    // cv::imread默认以BGR顺序读取彩色图像
    cv::Mat color_image = cv::imread(image_path, cv::IMREAD_COLOR);

    // 检查图像是否成功读取
    if (color_image.empty())
    {
        std::cerr << "ERROR: Could not read image from: " << image_path << std::endl;
        return -1; // 图像读取失败，退出程序
    }

    // 显示原始彩色图像
    cv::imshow("Original Color Image", color_image);

    // 将彩色图像转换为灰度图像
    cv::Mat gray_image;
    cv::cvtColor(color_image, gray_image, cv::COLOR_BGR2GRAY);

    // 对灰度图像进行二值化处理
    // cv::THRESH_BINARY 表示大于阈值的像素设为255，否则设为0
    cv::Mat binary_image;
    cv::threshold(gray_image, binary_image, binary_threshold, 255, cv::THRESH_BINARY);

    // 显示二值化后的图像
    cv::imshow("Binary Image", binary_image);

    // 等待用户按下任意键，窗口才会关闭
    cv::waitKey(0);

    return 0; // 程序成功执行
}