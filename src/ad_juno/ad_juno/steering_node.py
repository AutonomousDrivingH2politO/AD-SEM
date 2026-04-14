#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import Float64, Bool
from shared_objects.Motor import Stepper
from shared_objects.ROS_utils import Topics
from pathlib import Path

# Parametri di conversione e limiti
TRANSMISSION_RATIO = 14
MAX_ANGLE = 10
STEP_ANGLE = 1.8  # in gradi



class SteeringNode(Node):
    def __init__(self):
        super().__init__("steering_node")

        # Inizializza lo stepper con i parametri necessari
        self.steering = Stepper(
            interface="can",
            data_rate=1000000,
            module_id=3,  # modulo dello sterzo
            max_velocity=80_000,
            max_acc=50_000,
            MaxDeceleration=50_000,  # parametro di esempio: regola se necessario
            V1=100_000,
            A1=50_000,  # parametro di esempio: regola se necessario
            D1=50_000,  # parametro di esempio: regola se necessario
        )


        # Variabile per memorizzare il target (in gradi) ricevuto via ROS
        self.latest_angle = 0.0

        self.declare_parameter(
            "debug_root",
            "/home/bylogix/AD-SEM/DEBUG",  # default path
        )
        self.debug = (
            self.declare_parameter("debug", False).get_parameter_value().bool_value
        )
        self.debug_root = Path(
            self.get_parameter("debug_root").get_parameter_value().string_value
        )
        self.debug_root.mkdir(parents=True, exist_ok=True)

        # Create a timestamped log file for PID logs
        # timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
        # counter =0
        # counter+=1
        # self.log_file = self.debug_root / timestamp / f"pid_log_{counter}.txt"

        # self.get_logger().info(f"PID logs is in -> {self.log_file}")

        # QoS per il subscriber
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        topics = Topics()
        topic_names = topics.topic_names

        self.subscription = self.create_subscription(
            Float64, topic_names["steering"], self.callback_stepangle, qos_profile
        )

        self._stopped = False

        self.stop_sub = self.create_subscription(Bool , topic_names['stop'], self.callback_stop, qos_profile )


        self.get_logger().info("Steering node initialized (direct move_to, firmware ramp)")
    
    def callback_stepangle(self, msg):
        if self._stopped:
            return
        
        angle = msg.data

        # Saturazione
        if angle > MAX_ANGLE:
            angle = MAX_ANGLE
        elif angle < -MAX_ANGLE:
            angle = -MAX_ANGLE

        self.latest_angle = angle
        target_position = self.convert_angle_to_position(angle)

        if self.debug:
            self.get_logger().info(
            f"angle={angle:.2f}°, target_position={target_position}"
        )

        self.steering.move_stepper(target_position)

    def callback_stop(self, msg):
        self._stopped = msg.data
        if self._stopped:
            self.get_logger().info("Stop received — steering frozen")
            # Optionally: command stepper to hold current position
            # self.steering.move_stepper(self.convert_angle_to_position(0.0))
        else:
            self.get_logger().info("Stop released — steering resumed")

    def convert_angle_to_position(self, angle):
        """
        Converte l'angolo target (in gradi) in posizione (in microstep) coerente con il motore.
        """
        # Calcola il numero di step: ogni step è STEP_ANGLE gradi, convertito in microstep (256 microstep per step)
        steps = (angle / STEP_ANGLE) * 256
        # Applica il rapporto di trasmissione; il segno meno può essere necessario per la corretta direzione
        return -int(steps * TRANSMISSION_RATIO)

    
def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Steering node shutting down")
    finally:
        node.steering.disconnect_motor()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
