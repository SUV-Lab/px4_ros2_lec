import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge

bridge = CvBridge() 

class ImageSubscriber(Node) :
    def __init__(self) :
        super().__init__('image_sub')
        qos = QoSProfile(depth=10)
        
        self.last_val = ""
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', qos)    

        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback,
            qos)
        self.y_val = 0
        self.change = 0
        self.image = np.empty(shape=[1])


    def image_callback(self, data) :
        self.image = bridge.imgmsg_to_cv2(data, 'mono8')
        last_img_line = self.image[len(self.image) - 1]
        tt_size = len(last_img_line)
        real_center = tt_size // 2
        start_in = 0
        end_in = 0
        ok = False
        for i in range(0,len(last_img_line)):
            if last_img_line[i] < 100:
                if ok == False:
                    start_in = i
                    ok = True

            elif ok == True and last_img_line[i] > 100:
                ok = "end"
                end_in = i

        if ok == True:  
            end_in = len(last_img_line) - 1
        line_center = start_in + ((end_in - start_in) //2)
        
        cmd = Twist()
        if(line_center >= real_center - 15 and line_center <= real_center + 15):    
            cmd.linear.x = 0.1
            cmd_str = "y:0.1"
        
        elif(self.y_val >= 300 and self.y_val <= 900 and start_in == 0 and end_in == 0):
            self.change = self.change + 1

            if self.last_val == "z:-0.04":
                cmd.angular.z = 0.04
                cmd_str = "z:-0.04"
            else:
                cmd.angular.z = -0.04
                cmd_str = "z:-0.04"
        elif(line_center >= real_center):
            cmd.angular.z = -0.04
            cmd_str = "z:-0.04"
            self.change = 0
        else:
            cmd.angular.z = 0.04
            cmd_str = "z:0.04"
            self.change = 0


        if(self.last_val != cmd_str or self.change == 1):
            if(self.change != True):
                self.y_val = 0
            self.last_val = cmd_str
            self.cmd_vel_publisher.publish(cmd)

        if(self.last_val == "z:-0.04" or self.last_val == "z:0.04"):
            self.y_val = self.y_val + 1

        cv2.imshow('img', self.image)
        cv2.waitKey(100)
     
def main(args=None) :
  rclpy.init(args=args)
  node = ImageSubscriber()

  try :
    rclpy.spin(node)
  except KeyboardInterrupt :
    node.get_logger().info('Stopped by Keyboard')
  finally :
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__' :
  main()