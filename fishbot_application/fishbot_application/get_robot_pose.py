import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformListener,Buffer   # 坐标监听器
from tf_transformations import euler_from_quaternion  # 四元数转欧拉角
import math   # 提供角度转弧度等数学函数

class TFBroadcaster(Node):
    def __init__(self):
        # 初始化节点，名称为 "tf_broadcaster"
        super().__init__('tf_broadcaster')

        # 创建缓冲区Buffer
        self.buffer_=Buffer()

        # 创建监听器，把监听到的关系放在Buffer里面
        self.listener_ = TransformListener(self.buffer_,self)

        # 创建定时器，每隔1秒调用获取坐标关系方法
        self.timer_=self.create_timer(1,self.get_transform)

    def get_transform(self):
        """
        实时查询定时获取坐标关系 buffer_
        """
        
        try:
            result=self.buffer_.lookup_transform('map','base_footprint',
                rclpy.time.Time(seconds=0.0),rclpy.time.Duration(seconds=1.0))
            transform=result.transform
            self.get_logger().info(f'平移：{transform.translation}')
            self.get_logger().info(f'旋转（四元数）{transform.rotation}')
            # 四元数转欧拉角
            rotation_euler=euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ])
            self.get_logger().info(f'旋转（欧拉角）{rotation_euler}')

        # 捕获异常
        except Exception as e:
            # 打印异常
            self.get_logger().warn(f'获取坐标变换失败：原因{str(e)}')

def main():
    rclpy.init()
    node = TFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


