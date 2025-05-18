#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import LaserScan, Imu, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time


class ControleRobo(Node):

    def __init__(self):
        super().__init__('controle_robo_novo')

        self.bridge = CvBridge()

        # Estado da câmera
        self.bandeira_x = None
        self.largura_img = None
        self.perto_da_bandeira = False

        # Estado de navegação
        self.obstaculo_a_frente = False
        self.girar_para_esquerda = True
        self.distancia_frontal = 10.0

        # Controle de desvio
        self.desvio_em_andamento = False
        self.inicio_desvio = 0.0
        self.duracao_desvio = 1.0
        self.media_esq = 0.0
        self.media_dir = 0.0


        # Publishers e Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/robot_cam', self.camera_callback, 10)

        self.timer = self.create_timer(0.1, self.move_robot)

    def scan_callback(self, msg: LaserScan):
        num_ranges = len(msg.ranges)
        if num_ranges == 0:
            return

        # Faixa frontal
        indices_frente = list(range(330, 360)) + list(range(0, 31))
        dist_frente = [msg.ranges[i] for i in indices_frente if not np.isnan(msg.ranges[i])]
        self.obstaculo_a_frente = min(dist_frente) < 0.5 if dist_frente else False

        # Laterais
        indices_esq = list(range(60, 90))
        indices_dir = list(range(270, 300))
        dist_esq = [msg.ranges[i] for i in indices_esq if not np.isnan(msg.ranges[i])]
        dist_dir = [msg.ranges[i] for i in indices_dir if not np.isnan(msg.ranges[i])]

        media_esq = np.mean(dist_esq) if dist_esq else 0.0
        media_dir = np.mean(dist_dir) if dist_dir else 0.0

        if not self.desvio_em_andamento:
            self.girar_para_esquerda = media_esq > media_dir

        # Medir distância exata à frente
        indices_leque_frontal = list(range(320, 360)) + list(range(0, 41))
        distancias_leque = [msg.ranges[i] for i in indices_leque_frontal if not np.isnan(msg.ranges[i])]

        if distancias_leque:
            self.distancia_frontal = min(distancias_leque)

    def imu_callback(self, msg: Imu):
        pass

    def odom_callback(self, msg: Odometry):
        pass

    def camera_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Erro ao converter imagem: {e}')
            return

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (160, 100, 100), (180, 255, 255))
        mask = mask1 | mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            maior = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(maior)
            self.bandeira_x = x + w // 2
            self.largura_img = img.shape[1]
            self.perto_da_bandeira = w > 100
        else:
            self.bandeira_x = None
            self.perto_da_bandeira = False

    def move_robot(self):
        twist = Twist()
        tempo_atual = time.time()

        if self.perto_da_bandeira:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info('🚩 Parado em frente à bandeira!')

        elif self.obstaculo_a_frente and not self.perto_da_bandeira:
            if not self.desvio_em_andamento:
                self.inicio_desvio = tempo_atual
                self.desvio_em_andamento = True

                # Decide o lado do desvio com lógica mais esperta
                if self.bandeira_x is not None and self.largura_img:
                    centro = self.largura_img // 2
                    bandeira_esq = self.bandeira_x < centro
                    lado_obstaculo = "esquerda" if self.media_esq < self.media_dir else "direita"

                    # Se a bandeira está do lado mais obstruído, desvia pro outro lado
                    if (bandeira_esq and self.media_esq < self.media_dir) or (not bandeira_esq and self.media_dir < self.media_esq):
                        self.girar_para_esquerda = not bandeira_esq
                        self.get_logger().info(f'⚠️ Bandeira está no lado obstruído → desviando para {"DIREITA" if not bandeira_esq else "ESQUERDA"}')
                    else:
                        # Se o lado da bandeira é mais livre, mantém o giro pra ela
                        self.girar_para_esquerda = bandeira_esq
                        self.get_logger().info(f'✅ Lado da bandeira está livre → desviando para {"ESQUERDA" if bandeira_esq else "DIREITA"}')

            if tempo_atual - self.inicio_desvio < self.duracao_desvio:
                twist.angular.z = 0.4 if self.girar_para_esquerda else -0.4
                twist.linear.x = 0.0  # Se quiser um leve avanço, use 0.02
                self.get_logger().info(f'↩️ Desviando {"ESQUERDA" if self.girar_para_esquerda else "DIREITA"}...')
                self.cmd_vel_pub.publish(twist)
                return
            else:
                self.desvio_em_andamento = False

        elif self.bandeira_x is not None and self.largura_img:
            erro = self.bandeira_x - self.largura_img // 2
            if abs(erro) < 30:
                twist.linear.x = 0.1
                self.get_logger().info('🎯 Alinhado com a bandeira – avançando')
            elif erro < 0:
                twist.angular.z = 0.3
                self.get_logger().info('↪️ Girando para ESQUERDA (bandeira)')
            else:
                twist.angular.z = -0.3
                self.get_logger().info('↩️ Girando para DIREITA (bandeira)')

        else:
            twist.linear.x = 0.15
            self.get_logger().info('🔍 Explorando o ambiente...')

        self.cmd_vel_pub.publish(twist)



def main(args=None):
    rclpy.init(args=args)
    node = ControleRobo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
