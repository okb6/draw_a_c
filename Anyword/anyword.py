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
        self.length_of_word = len(self.Statement)
        self.letters = []
        self.get_letters()

        self.
        


    def get_letters(self):
        i = 0
        while i < self.length_of_word:
            individual = self.Statement[i]
            if individual == "":
                individual = "/"
            self.letters.append(individual)
            i += 1

    def get_details(self):
        i = 0
        self.words = []
        while i < self.lenth_of_word:
            

def main(args=None):
rclpy.init(args=args)
node = Anyword()
rclpy.spin(node)
rclpy.shutdown()

if __name__ == '__main__':
    main()
            