import rclpy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from rclpy.node import Node
import numpy

class Anyword(Node):
    super().__init__('anyword')
    def __init__(self):
        self.Statement = input("enter the statement or word you want to draw")
        self.dictionary = {'0':'first', '1':'second', '2':'third', '3':'fourth', '4':'fifth', '5':'sixth', '6':'seventh', '7':'eighth', '8':'ninth', '9':'tenth', '10':'eleventh'}
        self.length_of_word = len(self.Statement)

        self.initialize()

        self.twist_pub = self.create_publisher(Twist, 'Loki/twist_mux/cmd_vel', 1)

        self.starting_position = [0.0, 0.0]
        self.starting_angle = 0
        self.current_angle = 0

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.dt = (self.get_clock().now().nanoseconds) * pow(10,-9)

        while rclpy.ok():
            self.calculate_commands()


        
    def initialize(self):
        i = 0
        while i < self.length_of_word:
            
            word = self.statement[i]
            str_point = "%s.points" (word)

            self.declare_parameter(str_point, rclpy.Parameter.Type.INTEGER)
            points = self.get_parameter(str_point).value

            self.point_list.append[points]

            k = 0
            self.letter_angle = []
            self.letter_point = []
            self.use_points = []
            self.use_coord = {}
            self.use_angle = {}
            while k < points:
                prefix = self.dictionary["{}".format(i)]
                str_angle = "%s.%s_angle" (word, prefix)
                str_positions = "%s.%s_point" (word, prefix) 

                self.declare_parameter(str_angle, rclpy.Parameter.Type.INTEGER)
                angle = self.get_parameter(str_angle).value

                self.declare_parameter(str_positions, rclpy.Parameter.Type.DOUBLE_ARRAY)
                position = self.get_parameter(str_positions).value

                self.letter_angle.append(angle)
                self.letter_point.append(position)

                k += 1
            
            self.use_points.append(points)
            self.use_coord["{}".format(word)] = self.letter_point
            self.use_angle["{}".format(word)] = self.letter_angle

            i += 1

        return True

    def calculate_commands(self):
        wanted_position = [0.0, 0.0]
        i = 0
        while i < self.length_of_word:
            word = self.statement[i]
            k = 0
            for k < self.use_points[i]:
                if k == 0:
                    tolerance = 1e-1
                    wanted_angle = self.letter_angle["{}".format(word)][0]

                    if abs(wanted_angle - self.current_angle) < tolerance or abs(wanted_angle + self.current_angle) < tolerance:
                        k += 1
                        self.current_angle = wanted_angle
                        self.starting_position[0] = self.use_coord["{}".format(word)][0][0]
                        self.starting_position[1] = self.use_coord["{}".format(word)][0][1]
                    else:
                        if self.current_angle < wanted_angle:
                            neg = -1
                        else:
                            neg = 1
                        
                        self.vx = 0.0
                        self.vy = 0.0
                        self.wz = 1 * neg

                        t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                        self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                        self.current_angle += (self.wz * t)

                        twist_msg = Twist()

                        twist_msg.linear.x = self.vx * 5
                        twist_msg.linear.y = self.vy * 5
                        twist_msg.linear.z = 0.0
                        
                        twist_msg.angular.x = 0.0
                        twist_msg.angular.y = 0.0
                        twist_msg.angular.z = self.wz * 5

                        self.twist_pub.publish(twist_msg)

                else:
                    if abs(wanted_angle - self.current_angle) < tolerance or abs(wanted_angle + self.current_angle) < tolerance:
                        wanted_position[0] = self.use_coord["{}".format(word)][k][0]
                        wanted_position[1] = self.use_coord["{}".format(word)][k][1]

                        if (abs(wanted_position[0] - self.starting_position[0]) < tolerance or abs(wanted_position[0] + self.starting_position[0]) < tolerance) and (abs(wanted_position[1] - self.starting_position[1]) < tolerance or abs(wanted_position[1] + self.starting_position[1]) < tolerance):
                            k += 1
                                self.current_angle = wanted_angle
                                self.starting_position[0] = self.use_coord["{}".format(word)][k][0]
                                self.starting_position[1] = self.use_coord["{}".format(word)][k][1]
                        else:
                            if (wanted_position[0] - self.starting_position[0]) < 0:
                                negx = -1
                            elif (wanted_position[0] - self.starting_position[0]) < 0:
                                negx = 1
                            else:
                                negx = 0
                            
                            if (wanted_position[1] - self.starting_position[1]) > 0:
                                negy = -1
                            elif (wanted_position[1] - self.starting_position[1]) < 0:
                                negy = 1
                            else:
                                negy = 0

                            self.vx = 1.0 * negx
                            self.wz = 0.0
                            self.vy = 1.0 * negy
                            t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                            self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                            
                            self.starting_position[0] += (self.vx * t)
                            self.starting_position[1] += (self.vy * t)
                            
                            twist_msg = Twist()

                            twist_msg.linear.x = self.vx * 5
                            twist_msg.linear.y = self.vy * 5
                            twist_msg.linear.z = 0.0
                            
                            twist_msg.angular.x = 0.0
                            twist_msg.angular.y = 0.0
                            twist_msg.angular.z = self.wz * 5

                            self.twist_pub.publish(twist_msg)
                            
                                

                    else:
                        if self.current_angle < wanted_angle:
                            neg = -1
                        else:
                            neg = 1
                        
                        self.wz = 1 * neg

                        t = (self.get_clock().now().nanoseconds * pow(10,-9)) - self.dt
                        self.dt = self.get_clock().now().nanoseconds * pow(10,-9)
                        self.current_angle += (self.wz * t)

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
    node = Anyword()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
            