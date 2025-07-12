cmake_minimum_required(VERSION 3.8)
project(demo_threshold)

find_package(OpenCV REQUIRED)

# 添加你的源文件
add_executable(threshold
    main.cpp
)

# 链接 OpenCV 库
target_link_libraries(demo_threshold
    ${OpenCV_LIBS}
)
