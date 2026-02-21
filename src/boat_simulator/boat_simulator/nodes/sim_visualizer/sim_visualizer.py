import rclpy
from rclpy.node import Node
from custom_interfaces.msg import GPS
from custom_interfaces.msg import DesiredHeading
import matplotlib.pyplot as plt
import matplotlib.markers as mmarkers


class SimVisualizer(Node):
    def __init__(self):
        super().__init__("sim_visualizer")

        # GPS
        self.create_subscription(GPS, "/mock_gps", self.gps_callback, 10)

        # Latest GPS values
        self.__lat = None
        self.__lon = None

        # Heading
        self.__heading = 0.0
        self.create_subscription(DesiredHeading, "/desired_heading", self.heading_callback, 10)

        # Matplotlib setup
        plt.ion()
        self.__fig, self.__ax = plt.subplots()
        self.__ax.set_xlabel("Latitude")
        self.__ax.set_ylabel("Longitude")
        self.__ax.set_title("Boat Path")

        self.__x_data = []   # lat
        self.__y_data = []   # lon
        self.__line, = self.__ax.plot([], [], "-b")  # trail
        self.__dot, = self.__ax.plot([], [], marker="^", color="blue", markersize=12)

        # Timer for updating plot
        self.__timer = self.create_timer(0.1, self.update_plot)

    def gps_callback(self, msg):
        print("GPS:", msg.lat_lon.latitude, msg.lat_lon.longitude)
        self.__lat = msg.lat_lon.latitude
        self.__lon = msg.lat_lon.longitude

    def heading_callback(self, msg):
        # msg.heading.heading is the actual heading value
        self.__heading = msg.heading.heading


    def update_plot(self):
        if self.__lat is None or self.__lon is None:
            return

        # Append new point
        self.__x_data.append(self.__lat)
        self.__y_data.append(self.__lon)

        # Update trail + dot
        self.__line.set_xdata(self.__x_data)
        self.__line.set_ydata(self.__y_data)
        self.__dot.set_xdata([self.__lat])
        self.__dot.set_ydata([self.__lon])

        # Autoscale
        self.__ax.relim()
        self.__ax.autoscale_view()

        # Rotate the boat
        marker = mmarkers.MarkerStyle("^")
        marker._transform.rotate_deg(self.__heading)
        self.__dot.set_marker(marker)

        plt.draw()
        plt.pause(0.001)

def main():
    rclpy.init()
    node = SimVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
