import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class FKNode(Node):
    def __init__(self):
        super().__init__('fk_node')
        # Suscripción a los estados articulares
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        
    def dh_matrix(self, theta, d, a, alpha):
        return np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),               np.cos(alpha),              d],
            [0,              0,                           0,                          1]
        ])

    def listener_callback(self, msg):
        # Extraer las 6 articulaciones
        # Asegurar el orden de q1 a q6 buscando en msg.name si es necesario
        q = msg.position
        
        # Parámetros DH TM5-900 (theta, d, a, alpha)
        pi_2 = np.pi / 2
        A1 = self.dh_matrix(q[0], 0.1452, 0, -pi_2)
        A2 = self.dh_matrix(q[1] - pi_2, 0, 0.429, 0)
        A3 = self.dh_matrix(q[2], 0.1223, 0.4115, 0)
        A4 = self.dh_matrix(q[3] + pi_2, 0.106, 0, pi_2)
        A5 = self.dh_matrix(q[4], 0.11315, 0, -pi_2)
        A6 = self.dh_matrix(q[5], 0, 0, 0)
        
        # Transformación total de la base al efector
        T0_6 = A1 @ A2 @ A3 @ A4 @ A5 @ A6
        
        # Extracción de la posición cartesiana
        px, py, pz = T0_6[0,3], T0_6[1,3], T0_6[2,3]
        
        self.get_logger().info(f'Posición calculada FK: x={px:.4f}, y={py:.4f}, z={pz:.4f}')

def main(args=None):
    rclpy.init(args=args)
    node = FKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
