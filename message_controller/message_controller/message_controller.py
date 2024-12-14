import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
import pygame

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MessageController(Node):
    def __init__(self):
        super().__init__('message_controller_node')
        self.node = Node('mando')
        pygame.init()
        pygame.joystick.init()
        # Detectar joysticks conectados
        if pygame.joystick.get_count() == 0:
            
            self.get_logger().error("No se detectó ningún joystick conectado.")
            exit()

        # Usar el primer joystick detectado
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.node.get_logger().info(f"Joystick detectado: {self.joystick.get_name()}")
        
        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)

        
    def mando(self):
        try:
            while True:
                # Procesar eventos de pygame
                pygame.event.pump()
                deadzone = 0.15
                speed = 2
                # Leer los valores de los ejes (joysticks analógicos)
                axis_x = self.joystick.get_axis(0)  # horizontal 
                axis_y = self.joystick.get_axis(1)  # vertical
                target_speed = round(-axis_y,2) *speed 
                target_rotation = round(-axis_x ,2) * speed
                command_message = Twist()
                
                if abs(target_speed) > deadzone:
                    command_message.linear.x = target_speed
                else:
                    command_message.linear.x = 0.0

                # Aplicar el deadzone a la rotación angular
                if abs(target_rotation) > deadzone:
                    command_message.angular.z = target_rotation
                else:
                    command_message.angular.z = 0.0

                                              
                self.__publisher.publish(command_message)
                # Imprimir valores
                #self.node.get_logger().info(f"Ejes Izq: ({target_speed}, {target_rotation})")
            
        except KeyboardInterrupt:
            self.node.get_logger().info("\nFinalizando lectura...")
            self.joystick.quit()
            pygame.quit()    
    

def main(args=None):
    rclpy.init(args=args)
    
    message_controller = MessageController()
    try:
        message_controller.mando()
    except KeyboardInterrupt:
        message_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()