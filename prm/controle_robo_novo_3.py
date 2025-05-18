#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import cv2
import numpy as np


class ControleRobo(Node):
    def __init__(self):
        super().__init__('controle_robo_novo_3')

        # Publicador de velocidade
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscrições
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/robot_cam', self.camera_callback, 10)

        self.timer = self.create_timer(0.1, self.move_robot)

        # Estado interno
        self.bridge = CvBridge()
        self.obstaculo_a_frente = False
        self.girar_para_esquerda = True
        self.bandeira_x = None
        self.largura_img = None
        self.distancia_frontal = float('inf')
        self.chegou_na_bandeira = False


    def scan_callback(self, msg: LaserScan):
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # Leque frontal de -45° a +45°:
        indices_esq_frente = list(range(315, 360))  # -45° a 0°
        indices_dir_frente = list(range(0, 46))     # 0° a +45°

        dist_esq = [msg.ranges[i] for i in indices_esq_frente if not np.isnan(msg.ranges[i])]
        dist_dir = [msg.ranges[i] for i in indices_dir_frente if not np.isnan(msg.ranges[i])]

        min_esq = min(dist_esq) if dist_esq else float('inf')
        min_dir = min(dist_dir) if dist_dir else float('inf')
        media_esq = np.mean(dist_esq) if dist_esq else 0
        media_dir = np.mean(dist_dir) if dist_dir else 0

        if min(min_esq, min_dir) < 0.7:
            self.obstaculo_a_frente = True
            self.girar_para_esquerda = media_esq < media_dir

            direcao = "ESQUERDA" if self.girar_para_esquerda else "DIREITA"
            self.get_logger().info(
                f"[Desvio Frontal] Média esquerda: {media_esq:.2f} m | Média direita: {media_dir:.2f} m → Virando para {direcao}"
            )
        else:
            self.obstaculo_a_frente = False

    def camera_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.largura_img = cv_image.shape[1]

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Máscara para vermelho
        mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (160, 100, 100), (180, 255, 255))
        mask = mask1 | mask2

        # Encontrar os contornos da máscara (áreas vermelhas)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            maior = max(contours, key=cv2.contourArea)  # Encontrar o maior contorno (bandeira)
            M = cv2.moments(maior)
            if M["m00"] > 0:
                # Calcular a posição central da bandeira
                self.bandeira_x = int(M["m10"] / M["m00"])
                
                # Contar a quantidade de pixels vermelhos
                pixels_vermelhos = cv2.countNonZero(mask)  # Conta o número de pixels vermelhos na máscara
                self.get_logger().info(f"Pixels vermelhos detectados: {pixels_vermelhos}")
                
                # Definir um limite de pixels vermelhos para determinar se o robô está próximo
                if pixels_vermelhos > 1000:  # Ajuste esse valor conforme necessário
                    self.get_logger().info("Robô está perto da bandeira!")
                else:
                    self.get_logger().info("Robô ainda está se afastando da bandeira.")
            else:
                self.bandeira_x = None
        else:
            self.bandeira_x = None

    def imu_callback(self, msg: Imu):
        pass

    def odom_callback(self, msg: Odometry):
        pass

    def move_robot(self):
        twist = Twist()

        if self.bandeira_x is not None and self.largura_img:
            centro = self.largura_img // 2
            erro = self.bandeira_x - centro
            
            # Verificar proximidade com a bandeira com base nos pixels vermelhos detectados
            if self.bandeira_x is not None and self.largura_img:
                # Ajuste baseado na quantidade de pixels vermelhos detectados
                if self.pixels_vermelhos > 1000:  # Limite para considerar a bandeira próxima
                    self.get_logger().info("Robô chegou perto da bandeira! Parando...")
                    twist.linear.x = 0.0  # Parar o robô
                    twist.angular.z = 0.0
                else:
                    if not self.obstaculo_a_frente:
                        twist.linear.x = 0.15
                        twist.angular.z = -0.002 * erro  # Alinhamento suave com a bandeira
                        self.get_logger().info("Movendo em direção à bandeira 🎯")
                    else:
                        twist.linear.x = 0.15
                        twist.angular.z = 0.3 if self.girar_para_esquerda else -0.3
                        self.get_logger().info("Desviando de obstáculo durante perseguição 🚧")
        else:
            # Se a bandeira não for vista, o robô continua indo em frente
            if not self.obstaculo_a_frente:
                twist.linear.x = 0.1
                self.get_logger().info("Movendo reto sem bandeira 🚶")
            else:
                twist.linear.x = 0.03
                twist.angular.z = 0.15 if self.girar_para_esquerda else -0.15
                self.get_logger().info("Desviando de obstáculo sem bandeira 👀")

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()