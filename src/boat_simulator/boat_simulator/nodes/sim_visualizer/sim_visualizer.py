import rclpy
from rclpy.node import Node
from custom_interfaces.msg import GPS
from custom_interfaces.msg import DesiredHeading

import matplotlib
matplotlib.use("Agg")  # Must be before pyplot import
import matplotlib.pyplot as plt
import matplotlib.markers as mmarkers

import imageio
import io


class SimVisualizer(Node):
    def __init__(self):
        super().__init__("sim_visualizer")

        # GPS Subscription
        self.create_subscription(GPS, "/mock_gps", self.gps_callback, 10)

        # Latest GPS values
        self.__lat = None
        self.__lon = None

        # Heading Subscription
        self.__heading = 0.0
        self.create_subscription(DesiredHeading, "/desired_heading", self.heading_callback, 10)

        # Matplotlib setup
        self.__fig, self.__ax = plt.subplots()
        self.__ax.set_xlabel("Latitude")
        self.__ax.set_ylabel("Longitude")
        self.__ax.set_title("Boat Path")

        self.__x_data = []
        self.__y_data = []
        self.__frames = []
        self.__line, = self.__ax.plot([], [], "-b")
        self.__dot, = self.__ax.plot([], [], marker="^", color="blue", markersize=12)

        self.__timer = self.create_timer(0.1, self.update_plot)

    def gps_callback(self, msg):
        self.__lat = msg.lat_lon.latitude
        self.__lon = msg.lat_lon.longitude

    def heading_callback(self, msg):
        self.__heading = msg.heading.heading

    def update_plot(self):
        if self.__lat is None or self.__lon is None:
            return

        self.__x_data.append(self.__lat)
        self.__y_data.append(self.__lon)

        # Update elements
        self.__line.set_data(self.__x_data, self.__y_data)
        self.__dot.set_data([self.__lat], [self.__lon])

        # Rotate the boat marker
        marker = mmarkers.MarkerStyle("^")
        marker._transform.rotate_deg(self.__heading)
        self.__dot.set_marker(marker)

        self.__ax.relim()
        self.__ax.autoscale_view()

        # Capture frame
        tempImg = io.BytesIO()
        self.__fig.savefig(tempImg, format="png")
        tempImg.seek(0)
        self.__frames.append(imageio.v2.imread(tempImg))
        tempImg.close()

    def save_result(self):
        if self.__frames:
            self.get_logger().info(f"Saving {len(self.__frames)} frames to boat_path.gif")
            imageio.mimsave("boat_path.gif", self.__frames, fps=10)
            self.get_logger().info("Saved")


def main():
    rclpy.init()
    node = SimVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_result()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
