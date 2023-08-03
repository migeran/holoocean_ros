import holoocean
from hoveringauv_thruster_control.msg import ThrusterControl
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from queue import Queue
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sonar_data_processor import SonarDataProcessor
from std_msgs.msg import Header


#### CONSTANTS
SURFACE_DETECTION_THRESHOLD = 0.3
POINT_LENGTH = 12 # size of point in bytes

#### GET SONAR CONFIG
scenario = 'Dam-HoveringCustom'
config = holoocean.packagemanager.get_scenario(scenario)
config = config['agents'][0]['sensors'][-1]['configuration']
azimuth = config['Azimuth']
minimum_range = config['RangeMin']
maximum_range = config['RangeMax']
number_of_range_bins = config['RangeBins']
number_of_azimuth_bins = config['AzimuthBins']

#### GET PLOT READY
plt.ion()
fig, ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(8,5))
ax.set_theta_zero_location("N")
ax.set_thetamin(-azimuth/2)
ax.set_thetamax(azimuth/2)

theta = np.linspace(-azimuth/2, azimuth/2, number_of_azimuth_bins)*np.pi/180
r = np.linspace(minimum_range, maximum_range, number_of_range_bins)
T, R = np.meshgrid(theta, r)
z = np.zeros_like(T)

plt.grid(False)
plot = ax.pcolormesh(T, R, z, cmap='gray', shading='auto', vmin=0, vmax=1)
plt.tight_layout()
fig.canvas.draw()
fig.canvas.flush_events()

imaging_sonar_data_array = [] # the array with the diagram data
main_thread_operations_queue = Queue() # for callbacks that have to be called on th main thread.

#### LOAD SCENE
env = holoocean.make(scenario)

#### INITIALIZE COMMAND ARRAY
command = np.zeros(8)


#### NODE DEFINITIONS
class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')
        self.publisher_ = self.create_publisher(PointCloud2, 'point_cloud_topic', 3)

    def publish_pointcloud(self, pointcloud_data):
        pointcloud_message = PointCloud2()

        pointcloud_message.header = Header()
        pointcloud_message.header.stamp = self.get_clock().now().to_msg()
        pointcloud_message.header.frame_id = 'sonar_socket'

        pointcloud_message.height = 1

        number_of_points = len(pointcloud_data) / POINT_LENGTH

        if number_of_points == 0:
            self.get_logger().info('No points detected, skipping message.')
            return

        pointcloud_message.width = int(number_of_points)

        pointcloud_message.fields = []

        point_x_field = PointField()
        point_x_field.name = 'x'
        point_x_field.offset = 0
        point_x_field.datatype = PointField.FLOAT32
        point_x_field.count = 1
        pointcloud_message.fields.append(point_x_field)

        point_y_field = PointField()
        point_y_field.name = 'y'
        point_y_field.offset = 4
        point_y_field.datatype = PointField.FLOAT32
        point_y_field.count = 1
        pointcloud_message.fields.append(point_y_field)

        point_z_field = PointField()
        point_z_field.name = 'z'
        point_z_field.offset = 8
        point_z_field.datatype = PointField.FLOAT32
        point_z_field.count = 1
        pointcloud_message.fields.append(point_z_field)

        pointcloud_message.is_bigendian = False
        pointcloud_message.point_step = POINT_LENGTH
        pointcloud_message.data = pointcloud_data
        pointcloud_message.is_dense = True

        self.publisher_.publish(pointcloud_message)


#### START SONAR DATA CONVERSION THREAD
rclpy.init()
point_cloud_publisher = PointCloudPublisher()
sonar_data_processor = SonarDataProcessor(minimum_range, maximum_range, azimuth, SURFACE_DETECTION_THRESHOLD, point_cloud_publisher.publish_pointcloud)


#### NODE DEFINITIONS
class SimulatorNode(Node):
    def __init__(self):
        super().__init__('simulator')
        self.subscription = self.create_subscription(
            ThrusterControl,
            'thruster_control_topic',
            self.listener_callback,
            3)
        self.subscription  # prevent unused variable warning
        self.create_timer(0.016, self.timer_callback)   # capped 60 FPS

    def listener_callback(self, msg):
        global command
        command = msg.thruster_velocities

    def timer_callback(self):

        global command
        env.act('auv0', command)

        state = env.tick()

        if 'ImagingSonar' in state:
            imaging_sonar_data = state['ImagingSonar']

            global imaging_sonar_data_array
            imaging_sonar_data_array = imaging_sonar_data.ravel()

            main_thread_operations_queue.put(self.sonar_image_update_callback)

            global sonar_data_processor
            sonar_data_processor.process_data(imaging_sonar_data)

    def sonar_image_update_callback(self):
        plot.set_array(imaging_sonar_data_array)

        fig.canvas.draw()
        fig.canvas.flush_events()


#### MAIN FUNCTION
def main(args=None):
    simulator_node = SimulatorNode()

    executor = MultiThreadedExecutor()
    executor.add_node(simulator_node)
    global point_cloud_publisher
    executor.add_node(point_cloud_publisher)

    try:
        while True:
            executor.spin_once()
            while not main_thread_operations_queue.empty():
                operation = main_thread_operations_queue.get()
                operation()
    except KeyboardInterrupt:
        executor.shutdown()
        rclpy.shutdown()
        plt.ioff()
        plt.show()


if __name__ == '__main__':
    main()
