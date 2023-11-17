import rclpy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from rclpy.node import Node
import numpy

class DrawAC(Node):
    def __init__(self):
        super().__init__('draw_a_c')

        self.starting_position = [0.0 , 0.0]
        self.starting_angle = 0.0

        #Publishers
        self.twist_pub = self.create_publisher(Twist, 'Loki/twist_mux/cmd_vel', 1)

        self.starting_point = 0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.dt = (self.get_clock().now().nanoseconds) * pow(10,-9)

        while rclpy.ok():
            self.drawing_c()

    
    def drawing_c(self):
        if self.starting_point == 0:
            next_point = [1.0 , 0.0]
            dis_x = abs(next_point[0]) - abs(self.starting_position[0])
            dis_y = abs(next_point[1]) - abs(self.starting_position[1])

            if dis_x == 0 or dis_x < 0:
                self.starting_point += 1
                self.starting_position[0] = next_point[0]
                self.starting_position[1] = next_point[1]
                self.get_logger().info("first point has finished")
            else:
                self.vx = 1.0
                self.wz = 0.0
                self.vy = 0.0

                t = ((self.get_clock().now().nanoseconds) * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_position[0] += (0.5 * t)

        elif self.starting_point == 1:
            next_point = [2.0,-1.0]
            projected_angle = -(math.pi/6)
            dis_x = abs(next_point[0]) - abs(self.starting_position[0])
            dis_y = abs(next_point[1]) - abs(self.starting_position[1])

            if self.starting_angle >= projected_angle:
                self.wz = -1.0
                self.vx = 0.0
                self.vy = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_angle += (-0.333 * t)
            elif dis_x == 0.0 or dis_x < 0.0:
                self.starting_point += 1
                self.starting_position[0] = next_point[0]
                self.starting_position[1] = next_point[1]
                self.get_logger().info("Second point has finished")
            else:
                self.vx = 1.0
                self.wz = 0.0
                self.vy = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_position[0] += (self.vx * math.cos((self.starting_angle))) * t 
                self.starting_position[0] += (self.vx * math.sin((self.starting_angle))) * t
        
        elif self.starting_point == 2:
            next_point = [2.0,-8.0]
            projected_angle = 0.0
            dis_x = abs(next_point[0]) - abs(self.starting_position[0])
            dis_y = abs(next_point[1]) - abs(self.starting_position[1])
            
            if self.starting_angle <+ projected_angle:
                self.wz = 1.0
                self.vx = 0.0
                self.vy = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_angle += (0.45 * t)

            elif dis_y == 0.0 or dis_y < 0.0:
                self.starting_point += 1
                self.starting_position[0] = next_point[0]
                self.starting_position[1] = next_point[1]
                self.get_logger().info("Third point has finished")
            else:
                self.vy = 1.0
                self.vx = 0.0
                self.wz = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_position[1] += (1.0 * t)

        elif self.starting_point == 3:
            next_point = [1.0, -9.0]
            projected_angle = math.pi/6
            dis_x = self.starting_position[0] - next_point[0]
            dis_y = abs(next_point[1]) - abs(self.starting_position[1])

            if self.starting_angle <= projected_angle:
                self.wz = 1.0
                self.vx = 0.0
                self.vy = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_angle += (0.333 * t)
            elif dis_x == 0.0 or dis_x < 0.0:
                self.starting_point += 1
                self.starting_position[0] = next_point[0]
                self.starting_position[1] = next_point[1]
                self.get_logger().info("Fourth point has finished")
            else:
                self.vx = -1.0
                self.wz = 0.0
                self.vy = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_position[0] += (-0.33 * math.cos(self.starting_angle)) * t 
                # self.get_logger().info("{}".format(self.starting_position[0]))
                self.starting_position[1] += (self.vy * math.sin(self.starting_angle)) * t

        elif self.starting_point == 4:
            next_point = [-11.0, -9.0]
            projected_angle = 0.0
            dis_x = abs(next_point[0]) - abs(self.starting_position[0])
            dis_y = abs(next_point[1]) - abs(self.starting_position[1])

            if self.starting_angle >= projected_angle:
                self.wz = -1.0
                self.vx = 0.0
                self.vy = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_angle += (-0.30 * t)

            elif dis_x == 0.0 or dis_x < 0.0:
                self.starting_point += 1
                self.starting_position[0] = next_point[0]
                self.starting_position[1] = next_point[1]
                self.get_logger().info("Fifth point has finished")

            else:
                self.vx = -1.0
                self.wz = 0.0
                self.vy = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_position[0] += (0.66 * t)

        elif self.starting_point == 5:
            next_point = [-12.0, -8.0]
            projected_angle = -(math.pi/6)
            dis_x = abs(next_point[0]) - abs(self.starting_position[0])
            dis_y = abs(next_point[1]) - abs(self.starting_position[1])

            if self.starting_angle >= projected_angle:
                self.wz = -1.0
                self.vx = 0.0
                self.vy = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_angle += (-0.30 * t)

            elif dis_x == 0.0 or dis_x < 0.0:
                self.starting_point += 1
                self.starting_position[0] = next_point[0]
                self.starting_position[1] = next_point[1]
                self.get_logger().info("sixth point has finished")

            else:
                self.vx = -1.0
                self.wz = 0.0
                self.vy = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_position[0] += (self.vx * math.cos(self.starting_angle)) * t 
                self.starting_position[0] += (self.vx * math.sin(self.starting_angle)) * t

        elif self.starting_point == 6:
            next_point = [-12.0, -1.0]
            projected_angle = 0.0
            dis_x = abs(self.starting_position[0]) - abs(next_point[0])
            dis_y = abs(self.starting_position[1]) - abs(next_point[1])


            if self.starting_angle <= projected_angle:
                self.wz = 1.0
                self.vx = 0.0
                self.vy = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_angle += (0.4 * t)
            elif dis_y == 0.0 or dis_y < 0.0:
                self.starting_point += 1
                self.starting_position[0] = next_point[0]
                self.starting_position[1] = next_point[1]
                self.get_logger().info("seventh point has finished")
            else:
                self.vy = -1.0
                self.vx = 0.0
                self.wz = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_position[1] += (-self.vy/1.5 * t)

        elif self.starting_point == 7:
            next_point = [-11.0, 0.0]
            projected_angle = math.pi / 6
            dis_x = abs(self.starting_position[0]) - abs(next_point[0])
            dis_y = abs(next_point[1]) - abs(self.starting_position[1])

            if self.starting_angle <= projected_angle:
                
                self.wz = 1.0
                self.vx = 0.0
                self.vy = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_angle += (0.55 * t)
            elif dis_x == 0.0 or dis_x < 0.0:
                self.starting_point += 1
                self.starting_position[0] = next_point[0]
                self.starting_position[1] = next_point[1]
                self.get_logger().info("eighth point has finished")

            else:
                self.vx = 1.0
                self.wz = 0.0
                self.vy = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_position[0] += (0.5 * math.cos(self.starting_angle)) * t 
                self.get_logger().info("HELLO")
                self.starting_position[0] += (self.vx * math.sin(self.starting_angle)) * t

        elif self.starting_point == 8:
            next_point = [-10.0, 0.0]
            projected_angle = 0.0
            dis_x = abs(self.starting_position[0]) - abs(next_point[0])
            dis_y = abs(next_point[1]) - abs(self.starting_position[1])

            if self.starting_angle >= projected_angle:
                
                self.wz = -1.0
                self.vx = 0.0
                self.vy = 0.0

                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_angle += (-0.333 * t)

            elif dis_x == 0.0 or dis_x < 0.0:
                self.starting_point += 1
                self.starting_position[0] = next_point[0]
                self.starting_position[1] = next_point[1]
                self.get_logger().info("ninth point has finished")
            
            else:
                self.vx = 1.0
                self.wz = 0.0
                self.vy = 0.0
                t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                self.starting_position[0] += (0.5 * t)

        elif self.starting_point == 9:
            self.get_logger().info("CONGRATS YOU HAVE NOW WRITTEN A C")
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
            self.starting_point += 1


                    
                



                   
        
        
        twist_msg = Twist()

        twist_msg.linear.x = self.vx * 5
        twist_msg.linear.y = self.vy * 5
        twist_msg.linear.z = 0.0
        
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = self.wz * 5

        self.twist_pub.publish(twist_msg)




        

        
def main(args=None):
    rclpy.init(args=args)
    node = DrawAC()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()