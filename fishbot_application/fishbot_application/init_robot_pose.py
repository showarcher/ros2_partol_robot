from geometry_msgs.msg import PoseStamped 
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

def main():
    # 初始化 ROS2 通信
    rclpy.init()

    # 创建导航器对象（内部会启动一个节点）
    nav = BasicNavigator()

    # 定义初始位姿消息
    init_pose = PoseStamped()

    # 设置坐标系为 map（初始位姿必须在 map 坐标系下）
    init_pose.header.frame_id = "map"
    # 设置时间戳，使用导航器的时钟
    init_pose.header.stamp = nav.get_clock().now().to_msg()

    # 设置初始位置 (x=0, y=0)
    init_pose.pose.position.x = 0.0
    init_pose.pose.position.y = 0.0
    # 设置初始朝向（w=1 表示没有旋转）
    init_pose.pose.orientation.w = 1.0

    # 将初始位姿发送给导航模块（相当于 /initialpose）
    nav.setInitialPose(init_pose)

    # 等待 Nav2 完全启动（AMCL 定位、地图服务等就绪）
    nav.waitUntilNav2Active()

    # 保持节点运行（监听和处理回调）
    rclpy.spin(nav)

    # 关闭 ROS2 节点
    rclpy.shutdown()
