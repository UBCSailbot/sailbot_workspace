import custom_interfaces.msg as ci
import local_pathfinding.wind_coord_systems as wcs

MEAN_SPEED = ci.HelperSpeed(speed=15.0)  # mean boat speed in kmph,
START_POINT = ci.HelperLatLon(
    latitude=49.28, longitude=-123.185032
)  # Starting location of the mock
START_HEADING = ci.HelperHeading(heading=180.0)  # in degrees, heading of the boat
TW_SPEED_KMPH = 10.0
TW_DIRECTION_DEG = 90


def convert_tw_to_aw_in_boat_coord(tw_dir_deg, tw_speed_kmph, boat_heading_deg, boat_speed):
    """
    Helper function that computes aw from tw

    :param tw_dir_deg: Description
    :param tw_speed_kmph: Description
    :param boat_heading_deg: Description
    :param boat_speed: Description
    """
    aw_dir_deg, aw_speed_kmph = wcs.get_apparent_wind(
        tw_dir_deg,
        tw_speed_kmph,
        boat_heading_deg,
        boat_speed,
        ret_rad=False
    )

    aw_dir_boat_coord_deg = wcs.global_to_boat_coordinate(
        boat_heading_deg,
        aw_dir_deg
    )

    return aw_dir_boat_coord_deg, aw_speed_kmph


"""
For further tests:
START_POINT = ci.HelperLatLon(latitude=49.308157, longitude=-123.244801)
Change last line of mock_global_path.csv to: 49.289686,-123.195877
"""
