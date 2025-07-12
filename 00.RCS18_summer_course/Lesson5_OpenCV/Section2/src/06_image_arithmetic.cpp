#include <opencv2/opencv.hpp>
#include <iostream>

int main()
{
    // 创建纯色图：大小200x200，3通道，BGR格式
    cv::Mat red = cv::Mat(200, 200, CV_8UC3, cv::Scalar(0, 0, 255));   // 红色
    cv::Mat green = cv::Mat(200, 200, CV_8UC3, cv::Scalar(0, 255, 0)); // 绿色
    cv::Mat blue = cv::Mat(200, 200, CV_8UC3, cv::Scalar(255, 0, 0));  // 蓝色

    // 图像加法（红 + 绿）
    cv::Mat addResult;
    cv::add(red, green, addResult);

    // 图像加权混合（红和蓝，权重0.7和0.3）
    cv::Mat blendResult;
    cv::addWeighted(red, 0.7, blue, 0.3, 0, blendResult);

    // 按位与（绿 & 蓝）
    cv::Mat andResult;
    cv::bitwise_and(green, blue, andResult);

    // 按位或（红 | 蓝）
    cv::Mat orResult;
    cv::bitwise_or(red, blue, orResult);

    // 按位异或（绿 ^ 红）
    cv::Mat xorResult;
    cv::bitwise_xor(green, red, xorResult);

    // 按位非（红）
    cv::Mat notResult;
    cv::bitwise_not(red, notResult);

    // 显示所有结果
    cv::imshow("Red", red);
    cv::imshow("Green", green);
    cv::imshow("Blue", blue);
    cv::imshow("Add (Red + Green)", addResult);
    cv::imshow("AddWeighted (0.7*Red + 0.3*Blue)", blendResult);
    cv::imshow("Bitwise AND (Green & Blue)", andResult);
    cv::imshow("Bitwise OR (Red | Blue)", orResult);
    cv::imshow("Bitwise XOR (Green ^ Red)", xorResult);
    cv::imshow("Bitwise NOT (Red)", notResult);

    std::cout << "按 ESC 键关闭所有窗口\n";
    while (true)
    {
        int key = cv::waitKey(0);
        if (key == 27) // ESC键
            break;
    }
    cv::destroyAllWindows();

    return 0;
}
