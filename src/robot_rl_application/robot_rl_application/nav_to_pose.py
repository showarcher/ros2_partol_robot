from geometry_msgs.msg import PoseStamped 
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

def main():
    # 目标化 ROS2 通信
    rclpy.init()

    # 创建导航器对象（内部会启动一个节点）
    nav = BasicNavigator()

    # 等待 Nav2 完全启动（AMCL 定位、地图服务等就绪）后，开始导航至目标点 goal_pose
    nav.waitUntilNav2Active()

    # 定义目标位姿消息
    goal_pose = PoseStamped()

    # 设置坐标系为 map（目标位姿必须在 map 坐标系下）
    goal_pose.header.frame_id = "map"

    # 设置时间戳，使用导航器的时钟
    goal_pose.header.stamp = nav.get_clock().now().to_msg()

    # 设置目标位置 (x=0, y=0)
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 1.0

    # 设置目标朝向（w=1 表示没有旋转）
    goal_pose.pose.orientation.w = 1.0

    # 将目标位姿发送给导航模块
    nav.goToPose(goal_pose)

    # 如果没有完成任务，即到达 goal_pose 进行反馈，打印剩余距离，还可进行超时处理：超时取消任务
    while not nav.isTaskComplete():
        feedback=nav.getFeedback()
        nav.get_logger().info(f'剩余距离：{feedback.distance_remaining}')
        # nav.cancelTask()

    # 获取导航结果
    result = nav.getResult()
    nav.get_logger().info(f'导航结果：{result}')

    # 保持节点运行（监听和处理回调）
    # rclpy.spin(nav)

    # 关闭 ROS2 节点
    # rclpy.shutdown()