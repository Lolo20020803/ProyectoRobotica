import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
import pygame



ACCELERATION_INCREMENT = 0.1  # Factor de incremento de velocidad
ROTATION_SPEED = 4.0  # Velocidad de rotación

class MessageController(Node):
    def __init__(self):
        super().__init__('message_controller_node')
        
        self.__target_speed = 0.0
        self.__target_rotation = 0.0
        
        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        self.create_subscription(String, 'message_controler', self.__message_callback, 1)
        
    def __message_callback(self, message):
      
      if message.data.lower() == "avanza":
                self.__target_speed = ACCELERATION_INCREMENT
      elif message.data.lower() == "detente":
                self.__target_speed = 0.0  # Detener el robot
                self.__target_rotation = 0.0  # Detener la rotación
      elif message.data.lower() == "retrocede":
                self.__target_speed = -ACCELERATION_INCREMENT
      elif message.data.lower() == "izquierda" :
                self.__target_speed = ACCELERATION_INCREMENT
                self.__target_rotation = -ROTATION_SPEED
      elif message.data.lower() == "derecha":
                self.__target_speed = ACCELERATION_INCREMENT
                self.__target_rotation = ROTATION_SPEED
                
      forward_speed = self.__target_speed
      angular_speed = self.__target_rotation 
      command_message = Twist()
      command_message.linear.x = forward_speed
      command_message.angular.z = angular_speed
      self.__publisher.publish(command_message)

    

def main(args=None):
    rclpy.init(args=args)
    
    message_controller = MessageController()
    rclpy.spin(message_controller)
    
    message_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()