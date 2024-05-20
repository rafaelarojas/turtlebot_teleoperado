import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Empty

class BringupNode(Node):
    def __init__(self):
        super().__init__('bringup_node')
        self.robot_ready_publisher = self.create_publisher(Bool, 'robot_ready', 10)
        self.timer = self.create_timer(1.0, self.check_robot_status)
        self.srv = self.create_service(Empty, 'shutdown_bringup', self.shutdown_bringup_service)
        self.get_logger().info('Nó de inicialização pronto')

    def check_robot_status(self):
        robot_ready = Bool()
        robot_ready.data = True
        self.robot_ready_publisher.publish(robot_ready)
        self.get_logger().info('Robô está pronto')

    def shutdown_bringup_service(self, request, response):
        self.get_logger().info('Serviço de encerramento chamado. Encerrando nó de inicialização...')
        rclpy.shutdown()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BringupNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
