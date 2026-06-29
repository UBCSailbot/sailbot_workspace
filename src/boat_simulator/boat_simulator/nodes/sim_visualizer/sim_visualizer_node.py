import matplotlib
import numpy as np
import rclpy
from rclpy.node import Node

from custom_interfaces.msg import GPS, DesiredHeading, SimWorldState

matplotlib.use("Agg")  # Must be before pyplot import
import io  # noqa

import imageio  # noqa
import matplotlib.markers as mmarkers  # noqa
import matplotlib.pyplot as plt  # noqa
from matplotlib.lines import Line2D  # noqa


class SimVisualizer(Node):
    def __init__(self):
        super().__init__("sim_visualizer")

        # GPS Subscription
        self.create_subscription(GPS, "/gps", self.gps_callback, 10)

        # Latest GPS values
        self.__lat = None
        self.__lon = None
        self.__heading = None

        # Heading Subscription
        self.__desired_heading = 0.0
        self.create_subscription(DesiredHeading, "/desired_heading", self.heading_callback, 10)

        # Kinematics Subscription
        self.__glo_vel_x = 0.0
        self.__glo_vel_y = 0.0
        self.create_subscription(SimWorldState, "/kinematics", self.kinematics_callback, 10)

        # Matplotlib setup with 2 subplots: path (top) and kinematics (bottom)
        self.__fig, (self.__ax, self.__ax_kinem) = plt.subplots(2, 1, figsize=(10, 8))
        self.__ax.set_xlabel("Latitude")
        self.__ax.set_ylabel("Longitude")
        self.__ax.set_title("Boat Path")

        self.__ax_kinem.set_xlabel("Time (iterations)")
        self.__ax_kinem.set_ylabel("Velocity (m/s)")
        self.__ax_kinem.set_title("Kinematics Over Time")

        self.__x_data = []
        self.__y_data = []
        self.__time_steps = []
        self.__vel_mag_data = []
        self.__frames = []
        (self.__line,) = self.__ax.plot([], [], "-b")
        (self.__dot,) = self.__ax.plot([], [], marker="^", color="blue", markersize=12)

        # Kinematics plot lines
        (self.__line_vel_mag,) = self.__ax_kinem.plot(
            [], [], "-r", label="Velocity Magnitude (m/s)"
        )
        self.__ax_kinem.legend(loc="upper left")

        self.__heading_arrow = None
        self.__desired_heading_arrow = None

        self.__ax.legend(
            handles=[
                Line2D([0], [0], color="blue", linewidth=2, label="Heading"),
                Line2D([0], [0], color="red", linewidth=2, label="Desired Heading"),
            ]
        )

        self.__timer = self.create_timer(0.1, self.update_plot)

    def gps_callback(self, msg):
        self.__lat = msg.lat_lon.latitude
        self.__lon = msg.lat_lon.longitude
        self.__heading = msg.heading.heading

    def heading_callback(self, msg):
        self.__desired_heading = msg.heading.heading

    def kinematics_callback(self, msg):
        self.get_logger().info(
            f"Received kinematics: Vel X={msg.global_pose.position.x:.2f} m/s, "
            f"Vel Y={msg.global_pose.position.y:.2f} m/s"
        )
        self.__glo_vel_x = msg.current_velocity.x
        self.__glo_vel_y = msg.current_velocity.y

    def update_plot(self):
        if self.__lat is None or self.__lon is None:
            return

        self.__x_data.append(self.__lat)
        self.__y_data.append(self.__lon)
        self.__time_steps.append(len(self.__time_steps))
        self.__vel_mag_data.append(np.hypot(self.__glo_vel_x, self.__glo_vel_y))

        # Update elements
        self.__line.set_data(self.__x_data, self.__y_data)
        self.__dot.set_data([self.__lat], [self.__lon])

        # Rotate the boat marker
        marker = mmarkers.MarkerStyle("^")
        marker._transform.rotate_deg(self.__heading)
        self.__dot.set_marker(marker)

        # Remove previous heading arrows
        if self.__heading_arrow:
            self.__heading_arrow.remove()
        if self.__desired_heading_arrow:
            self.__desired_heading_arrow.remove()

        # Scale arrow length to 10% of current track extent, with a minimum guard
        x_span = max(self.__x_data) - min(self.__x_data) if len(self.__x_data) > 1 else 0
        y_span = max(self.__y_data) - min(self.__y_data) if len(self.__y_data) > 1 else 0
        arrow_len = 0.1 * max(x_span, y_span, 1e-4)

        h_rad = np.deg2rad(self.__heading)
        self.__heading_arrow = self.__ax.quiver(
            self.__lat,
            self.__lon,
            np.cos(h_rad) * arrow_len,
            np.sin(h_rad) * arrow_len,
            color="blue",
            scale=1,
            scale_units="xy",
            angles="xy",
            width=0.005,
            headwidth=4,
            headlength=5,
        )

        dh_rad = np.deg2rad(self.__desired_heading)
        self.__desired_heading_arrow = self.__ax.quiver(
            self.__lat,
            self.__lon,
            np.cos(dh_rad) * arrow_len,
            np.sin(dh_rad) * arrow_len,
            color="red",
            scale=1,
            scale_units="xy",
            angles="xy",
            width=0.005,
            headwidth=4,
            headlength=5,
        )

        self.__ax.relim()
        self.__ax.autoscale_view()

        # Update kinematics plot
        self.__line_vel_mag.set_data(self.__time_steps, self.__vel_mag_data)
        self.__ax_kinem.relim()
        self.__ax_kinem.autoscale_view()

        # Capture frame
        tempImg = io.BytesIO()
        self.__fig.savefig(tempImg, format="png")
        tempImg.seek(0)
        self.__frames.append(imageio.imread(tempImg))
        tempImg.close()

    def save_result(self):
        if self.__frames:
            file_path = (
                "/workspaces/sailbot_workspace/src/boat_simulator/"
                "boat_simulator/nodes/sim_visualizer/boat_path.gif"
            )
            self.get_logger().info(f"Saving to {file_path}...")
            imageio.mimsave(file_path, self.__frames, fps=10)
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
