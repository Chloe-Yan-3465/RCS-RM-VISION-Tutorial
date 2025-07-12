# Ros2中的消息

## I.通信机制架构

## II.常见官方消息类型

实操要求：pkg create ，将ori的代码，改成节点，节点名为ImgNode，
声明发布者1：
```C++
sensor_msgs::msg::Image img_pub_; // 图像的ros2消息发布者
sensor_msgs::msg::Image img_sub_; // 图像ros2消息订阅者
```



## III.自定义消息类型
```msg
# Armor.msg
std_msgs/Header header
string frame_id
string armor_id
geometry_msgs/msg/PoseStamped posestamped
```

3份代码：  
img_demo 为imshow的ros2版  
auto_aim_interfaces 为自定义消息  
image_ws 为完整的pkg示例  
