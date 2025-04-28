import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

import sys
import tty
import termios
import threading
import time
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.current_pose = None
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    def pose_callback(self, msg):
        self.current_pose = msg

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def keyboard_listener(self):
        while rclpy.ok():
            key = self.get_key()
            if key.lower() == 'm':
                self.get_logger().info("Dibujando letra M")
                self.draw_M()
            elif key.lower() == 'f':
                self.get_logger().info("Dibujando letra F")
                self.draw_F()
            elif key.lower() == 'c':
                self.get_logger().info("Dibujando letra C")
                self.draw_C()
            elif key.lower() == 'd':
                self.get_logger().info("Dibujando letra D")
                self.draw_D()
            elif key.lower() == 'v':
                self.get_logger().info("Dibujando letra V")
                self.draw_V()
            elif key.lower() == 'q':
                self.get_logger().info("Dibujando letra Q")
                self.draw_Q()
            elif key == '\x03':  # Ctrl+C para salir
                break

    def teleport(self, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        future = self.teleport_client.call_async(req)
        start_time = time.time()
        while not future.done() and (time.time() - start_time < 2.0):
            time.sleep(0.1)
        if future.done():
            try:
                future.result()
            except Exception as e:
                self.get_logger().error(f"Error al teleportar: {e}")
        else:
            self.get_logger().error("Teleport no completado a tiempo")

    def move_distance(self, distance, speed=2.0):
        duration = abs(distance) / speed
        msg = Twist()
        msg.linear.x = speed if distance >= 0 else -speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        self.publisher_.publish(Twist())
        time.sleep(0.1)

    def turn_angle(self, angle, angular_speed=1.0):
        duration = abs(angle) / angular_speed
        msg = Twist()
        msg.angular.z = angular_speed if angle >= 0 else -angular_speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        self.publisher_.publish(Twist())
        time.sleep(0.1)

    def save_initial_pose(self):
        if self.current_pose is None:
            self.get_logger().error("Pose no disponible")
            return None
        return (self.current_pose.x, self.current_pose.y, self.current_pose.theta)

    def draw_M(self):
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial
        h = 3.0
        self.turn_angle(math.radians(90))
        self.move_distance(h)
        self.turn_angle(math.radians(-135))
        diag = h * math.sqrt(2) / 2
        self.move_distance(diag)
        self.turn_angle(math.radians(90))
        self.move_distance(diag)
        self.turn_angle(math.radians(-135))
        self.move_distance(h)
        self.teleport(x, y, theta)

    def draw_F(self):
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial
        h = 3.0
        width = 1.5
        self.turn_angle(math.radians(90))
        self.move_distance(h)
        self.turn_angle(math.radians(-90))
        self.move_distance(width)
        self.turn_angle(math.radians(180))
        self.move_distance(width)
        self.turn_angle(math.radians(90))
        self.move_distance(h / 2)
        self.turn_angle(math.radians(90))
        self.move_distance(width * 0.8)
        self.turn_angle(math.radians(180))
        self.move_distance(width * 0.8)
        self.turn_angle(math.radians(90))
        self.move_distance(h / 2)
        self.teleport(x, y, theta)

    def draw_C(self):
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial
        radius = 1.5
        angular_speed = 1.0  # rad/s
        linear_speed = angular_speed * radius  # Para movimiento circular
        
        # Ajuste: La "C" es un semicírculo de 180° hacia la izquierda
        self.turn_angle(math.radians(180))  # Orientar hacia el norte
        #self.move_distance(radius)  # Mover al punto de inicio del arco
        
        # Dibujar semicírculo (180°)
        start_angle = math.radians(180)
        duration = start_angle / angular_speed
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed  # Giro hacia la izquierda
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        
        self.publisher_.publish(Twist())
        self.teleport(x, y, theta)  # Volver al punto inicial

    def draw_D(self):
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial
        h = 3.0
        radius = h / 2
        angular_speed = 1.0
        linear_speed = radius * angular_speed
        self.turn_angle(math.radians(90))
        self.move_distance(h)
        self.turn_angle(math.radians(-90))
        duration = math.pi / angular_speed
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = -angular_speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        self.publisher_.publish(Twist())
        self.teleport(x, y, theta)

    def draw_V(self):
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial
        length = 3.0
        angle = 60  # Ángulo entre las dos líneas de la "V"
        
        # Ajuste: Dibujar desde el vértice hacia los extremos
        self.turn_angle(math.radians(120))  # Girar a la derecha
        self.move_distance(length)
        self.teleport(x, y, theta)  # Volver al centro
        
        self.turn_angle(math.radians(60))  # Girar a la izquierda
        self.move_distance(length)
        self.teleport(x, y, theta)  # Volver al centro
    def draw_Q(self):
        initial = self.save_initial_pose()
        if not initial:
            return
        x, y, theta = initial
        radius = 1.5
        tail_length = 1.0
        angular_speed = 1.0
        linear_speed = radius * angular_speed
        duration = 2 * math.pi / angular_speed
        msg = Twist()
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)
        self.publisher_.publish(Twist())
        self.turn_angle(math.radians(-45))
        self.move_distance(tail_length)
        self.teleport(x, y, theta)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()