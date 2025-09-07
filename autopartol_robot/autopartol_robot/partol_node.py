from geometry_msgs.msg import PoseStamped,Pose
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformListener,Buffer   # 坐标监听器
from tf_transformations import euler_from_quaternion,quaternion_from_euler  # 四元数转欧拉角,欧拉角转四元数
import math   # 提供角度转弧度等数学函数
from autopartol_interfaces.srv import SpeechText  # 语音合成服务的消息接口

# 图像转换与保存
from sensor_msgs.msg import Image   # 消息接口
from cv_bridge import CvBridge  # image_raw话题内容转换为图像
import cv2  # 保存图像到本地


# 一.定义巡检控制类:继承BasicNavigator,其继承于Node
class PatrolNode(BasicNavigator):
    def __init__(self, node_name='partol_node'):
        super().__init__(node_name)
        # 1.1声明相关参数-初始化机器人坐标,设置目标巡检点
        self.declare_parameter('initial_point',[0.0, 0.0, 0.0])
        self.declare_parameter('target_points',[0.0, 0.0, 0.0, 1.0, 1.0, 1.57])

        # 1.2图像保存参数设置
        self.declare_parameter('img_save_path','')

        # 1.3声明对应变量-初始化机器人坐标,设置目标巡检点
        self.initial_point_=self.get_parameter('initial_point').value
        self.target_points_=self.get_parameter('target_points').value
        self.img_save_path=self.get_parameter('img_save_path').value

        # 1.4设置监听者
        self.buffer_=Buffer()
        self.listener_=TransformListener(self.buffer_,self)

        # 1.4设置语音合成客户端：消息接口，合成文本
        self.speech_client_=self.create_client(SpeechText,'speech_text')

        # 1.5创建cv_bridge
        self.cv_bridge_=CvBridge()
        self.latest_img_=None      # 实时保存最新图片
        self.img_sub_=self.create_subscription(Image,'/camera_sensor/image_raw',self.img_callback,1)

# 二、图像转换相关函数
    # 1.创建图像的回调函数
    def img_callback(self,msg):
        self.latest_img_=msg

    # 2.图像转换及保存函数
    def record_img(self):
        if self.latest_img_ is not None:
            pose=self.get_current_pose()
            cv_image=self.cv_bridge_.imgmsg_to_cv2(self.latest_img_)
            cv2.imwrite(
                f'{self.img_save_path}img_{pose.translation.x:3.2f}_{pose.translation.y:3.2f}.png',
                cv_image
            )

# 三、语音播报相关函数
    # 1.通过xy和角度yaw返回一个PoseStamped对象
    def get_pose_by_xyyaw(self,x,y,yaw):
        pose=PoseStamped()
        pose.header.frame_id='map'
        pose.pose.position.x=x
        pose.pose.position.y=y
        # 返回顺序：xyzw
        quat=quaternion_from_euler(0,0,yaw)
        pose.pose.orientation.x=quat[0]
        pose.pose.orientation.y=quat[1]
        pose.pose.orientation.z=quat[2]
        pose.pose.orientation.w=quat[3]
        return pose

    # 2.初始化机器人的位姿
    def init_robot_pose(self):
        self.initial_point_=self.get_parameter('initial_point').value
        init_pose=self.get_pose_by_xyyaw(self.initial_point_[0],self.initial_point_[1],self.initial_point_[2])
        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()# 等待导航可用

    # 3.通过参数值获取目标点的集合
    def get_target_points(self):
        points=[]
        self.target_points_=self.get_parameter('target_points').value
        # 将 target_points 6元素的一维数组转换为二维数组:遍历+赋值
        for index in range(int(len(self.target_points_)/3)):
            base=index*3
            x=self.target_points_[base]
            y=self.target_points_[base+1]
            yaw=self.target_points_[base+2]
            points.append([x,y,yaw])
            self.get_logger().info(f"获取到目标点{index}->{x},{y},{yaw}")
        return points

    # 4.导航至目标点
    def nav_to_pose(self,target_point):
        self.goToPose(target_point)
        while not self.isTaskComplete():
            feedback=self.getFeedback()
            self.get_logger().info(f'剩余距离：{feedback.distance_remaining}')
        result=self.getResult()
        self.get_logger().info(f'导航结果：{result}')

    # 5.获取机器人当前位置
    def get_current_pose(self):
        while rclpy.ok():
            try:
                result=self.buffer_.lookup_transform('map','base_footprint',
                    rclpy.time.Time(seconds=0.0),rclpy.time.Duration(seconds=1.0))
                transform=result.transform
                self.get_logger().info(f'平移：{transform.translation}')
                return transform
            # 捕获异常
            except Exception as e:
                # 打印异常
                self.get_logger().warn(f'获取坐标变换失败：原因{str(e)}')

    # 6.调用语音合成服务
    def speech_text(self,text):
        # 检测语音合成服务是否上线
        while not self.speech_client_.wait_for_service(timeout_sec=1):
            self.get_logger().info(f'语音合成服务未上线，等待中…')

        # 创建请求
        request=SpeechText.Request()
        request.text=text
        # 异步调用服务，发送请求
        future=self.speech_client_.call_async(request)
        rclpy.spin_until_future_complete(self,future)
        if future.result() is not None:
            response=future.result()
            if response.result==True:
                self.get_logger().info(f'语音合成成功：{text}')
            else:
                self.get_logger().warn(f'语音合成失败：{text}')

        else:
            self.get_logger().warn(f'语音合成服务响应失败')


def main():

    rclpy.init()
    
    # 1.创建节点
    partol=PatrolNode()
    partol.speech_text('正在准备初始化位置')

    # 2.初始化机器人坐标
    partol.init_robot_pose()
    partol.speech_text('位置初始化完成')

    # 3.获取目标点坐标
    while rclpy.ok():
        points=partol.get_target_points()
        # 循环遍历
        for point in points:
            x,y,yaw=point
            target_pose=partol.get_pose_by_xyyaw(x,y,yaw)
            partol.speech_text(f'正在前往{x},{y}目标点')

    # 4.开始导航
            partol.nav_to_pose(target_pose)

    # 5.记录图像
            partol.speech_text(f'已经到达目标点{x},{y}，正在准备记录图像')
            partol.record_img()
            partol.speech_text(f'图像已记录完成')
            
    rclpy.shutdown()