import os

import post_server as ps
import pytest
from custom_interfaces.msg import HelperLatLon, Path

from local_pathfinding.coord_systems import GEODESIC, meters_to_km
from local_pathfinding.global_path import (
    _interpolate_path,
    calculate_interval_spacing,
    generate_path,
    get_most_recent_file,
    get_path,
    get_pos,
    interpolate_path,
    path_to_dict,
    post_path,
    write_to_file,
)


# ------------------------- TEST _INTERPOLATE_PATH -------------------------
@pytest.mark.parametrize(
    "pos,global_path,interval_spacing",
    [
        (
            HelperLatLon(latitude=47.00, longitude=122.00),
            Path(
                waypoints=[
                    HelperLatLon(latitude=48.00, longitude=123.00),
                    HelperLatLon(latitude=49.00, longitude=124.00),
                    HelperLatLon(latitude=50.00, longitude=125.00),
                ]
            ),
            30.0,
        )
    ],
)
def test__interpolate_path(
    pos: HelperLatLon,
    global_path: Path,
    interval_spacing: float,
):
    """Test the _interpolate_path method of MockGlobalPath.

    Args:
        global_path (HelperLatLon): The global path.
        interval_spacing (float): The desired spacing between waypoints.
        pos (HelperLatLon): The position of Sailbot.
    """

    path_spacing = calculate_interval_spacing(pos, global_path.waypoints)

    interpolated_path = _interpolate_path(
        global_path=global_path,
        interval_spacing=interval_spacing,
        pos=pos,
        path_spacing=path_spacing,
        write=False,
    )

    assert isinstance(interpolated_path, Path)

    # Ensure proper spacing between waypoints
    dists = calculate_interval_spacing(pos, interpolated_path.waypoints)

    assert max(dists) <= interval_spacing, "Interval spacing is not correct"


# ------------------------- TEST INTERVAL_SPACING -------------------------
@pytest.mark.parametrize(
    "pos,waypoints",
    [
        (
            HelperLatLon(latitude=48.95, longitude=123.56),
            [
                HelperLatLon(latitude=48.95, longitude=123.55),
            ],
        ),
        (
            HelperLatLon(latitude=48.95, longitude=123.56),
            [
                HelperLatLon(latitude=48.95, longitude=123.55),
                HelperLatLon(latitude=85.95, longitude=13.56),
            ],
        ),
        (
            HelperLatLon(latitude=48.95, longitude=123.56),
            [
                HelperLatLon(latitude=48.95, longitude=123.55),
                HelperLatLon(latitude=85.95, longitude=13.56),
                HelperLatLon(latitude=85.00, longitude=13.00),
            ],
        ),
    ],
)
def test_interval_spacing(pos: HelperLatLon, waypoints: list[HelperLatLon]):
    """Test the greatest_interval method of MockGlobalPath.

    Args:
        pos (HelperLatLon): The start position.
        waypoints (list[HelperLatLon]): The waypoints of the global path.
    """
    greatest_interval = max(calculate_interval_spacing(pos, waypoints))

    if len(waypoints) > 1:
        expected_interval = meters_to_km(
            GEODESIC.inv(
                lats1=waypoints[0].latitude,
                lons1=waypoints[0].longitude,
                lats2=waypoints[1].latitude,
                lons2=waypoints[1].longitude,
            )[2]
        )
    else:
        expected_interval = meters_to_km(
            GEODESIC.inv(
                lats1=pos.latitude,
                lons1=pos.longitude,
                lats2=waypoints[0].latitude,
                lons2=waypoints[0].longitude,
            )[2]
        )

    assert greatest_interval == pytest.approx(
        expected_interval
    ), "Greatest interval is not correct"


# ------------------------- TEST GENERATE_PATH -------------------------
@pytest.mark.parametrize(
    "pos,dest,interval_spacing",
    [
        (
            HelperLatLon(latitude=48.95, longitude=123.56),
            HelperLatLon(latitude=38.95, longitude=133.36),
            30.0,
        ),
        (
            HelperLatLon(latitude=-48.95, longitude=123.56),
            HelperLatLon(latitude=38.95, longitude=-133.36),
            20.0,
        ),
        (
            HelperLatLon(latitude=48.95, longitude=123.56),
            HelperLatLon(latitude=48.95, longitude=123.55),
            5.0,
        ),
    ],
)
def test_generate_path(
    pos: HelperLatLon,
    dest: HelperLatLon,
    interval_spacing: float,
):
    """Test the generate_path method of MockGlobalPath.

    Args:
        dest (HelperLatLon): The destination of the global path.
        pos (HelperLatLon): The position of Sailbot.
        interval_spacing (float): The desired spacing between waypoints.
    """
    global_path = generate_path(
        dest=dest,
        interval_spacing=interval_spacing,
        pos=pos,
    )

    assert isinstance(global_path, Path)

    if isinstance(dest, list):
        assert global_path.waypoints[-1].latitude == pytest.approx(
            dest[-1].latitude
        ), "final waypoint latitude is not correct"
        assert global_path.waypoints[-1].longitude == pytest.approx(
            dest[-1].longitude
        ), "final waypoint longitude is not correct"
    else:
        assert global_path.waypoints[-1].latitude == pytest.approx(
            expected=dest.latitude
        ), "final waypoint latitude is not correct"
        assert global_path.waypoints[-1].longitude == pytest.approx(
            expected=dest.longitude
        ), "final waypoint longitude is not correct"

    # Ensure proper spacing between waypoints
    for i in range(1, len(global_path.waypoints)):
        dist = GEODESIC.inv(
            global_path.waypoints[i - 1].longitude,
            global_path.waypoints[i - 1].latitude,
            global_path.waypoints[i].longitude,
            global_path.waypoints[i].latitude,
        )[2]
        dist *= 0.001  # convert to km
        assert dist <= interval_spacing, "Interval spacing is not correct"


# ------------------------- TEST GET_MOST_RECENT_FILE -------------------------
@pytest.mark.parametrize(
    "file_path,global_path,tmstmp",
    [
        (
            "/workspaces/sailbot_workspace/src/local_pathfinding/global_paths/test_file.csv",
            Path(),
            False,
        )
    ],
)
def test_get_most_recent_file(file_path: str, global_path: Path, tmstmp: bool):
    # create a file in the directory
    write_to_file(file_path=file_path, global_path=global_path, tmstmp=tmstmp)

    assert (
        get_most_recent_file(directory_path=file_path[: -len(file_path.split("/")[-1])])
        == file_path
    ), "Did not get most recent file"

    os.remove(file_path)


# ------------------------- TEST GET_PATH -------------------------
@pytest.mark.parametrize(
    "file_path",
    [("/workspaces/sailbot_workspace/src/local_pathfinding/global_paths/mock_global_path.csv")],
)
def test_get_path(file_path: str):
    """ "
    Args:
        file_path (str): The path to the global path csv file.
    """
    global_path = get_path(file_path)

    assert isinstance(global_path, Path)

    # Check that the path is formatted correctly
    for waypoint in global_path.waypoints:
        assert isinstance(waypoint, HelperLatLon), "Waypoint is not a HelperLatLon"
        assert isinstance(waypoint.latitude, float), "Waypoint latitude is not a float"
        assert isinstance(waypoint.longitude, float), "Waypoint longitude is not a float"


# ------------------------- TEST GET_POS -------------------------
@pytest.mark.parametrize(
    "pos", [HelperLatLon(latitude=49.34175775635472, longitude=-123.35453636335373)]
)
def test_get_pos(pos: HelperLatLon):
    """
    Args:
        pos (HelperLatLon): The position of the Sailbot.
    """

    pos = get_pos()
    assert pos is not None, "No position data received"
    assert pos.latitude is not None, "No latitude"
    assert pos.longitude is not None, "No longitude"


# ------------------------- TEST INTERPOLATE_PATH -------------------------
@pytest.mark.parametrize(
    "path,pos,interval_spacing",
    [
        (
            Path(
                waypoints=[
                    HelperLatLon(latitude=48.95, longitude=123.56),
                    HelperLatLon(latitude=38.95, longitude=133.36),
                    HelperLatLon(latitude=28.95, longitude=143.36),
                ]
            ),
            HelperLatLon(latitude=58.95, longitude=113.56),
            50.0,
        )
    ],
)
def test_interpolate_path(path: Path, pos: HelperLatLon, interval_spacing: float):
    """
    Args:
        path (Path): The global path.
        pos (HelperLatLon): The position of the Sailbot.
        interval_spacing (float): The spacing between each waypoint.
    """
    formatted_path = interpolate_path(
        path=path, pos=pos, interval_spacing=interval_spacing, file_path="", write=False
    )

    assert isinstance(formatted_path, Path), "Formatted path is not a Path"

    # Check that the path is formatted correctly
    for waypoint in formatted_path.waypoints:
        assert isinstance(waypoint, HelperLatLon), "Waypoint is not a HelperLatLon"
        assert isinstance(waypoint.latitude, float), "Waypoint latitude is not a float"
        assert isinstance(waypoint.longitude, float), "Waypoint longitude is not a float"

    path_spacing = calculate_interval_spacing(pos, formatted_path.waypoints)
    assert max(path_spacing) <= interval_spacing, "Path spacing is too large"
    assert max(path_spacing) <= interval_spacing, "Path spacing is too large"


# ------------------------- TEST PATH_TO_DICT -------------------------
@pytest.mark.parametrize(
    "path,expected",
    [
        (
            Path(
                waypoints=[
                    HelperLatLon(latitude=48.123446, longitude=123.123446),
                    HelperLatLon(latitude=38.123456, longitude=133.123456),
                ]
            ),
            {0: "(48.1234, 123.1234)", 1: "(38.1235, 133.1235)"},
        ),
    ],
)
def test_path_to_dict(path: Path, expected: dict[int, str]):
    path_dict = path_to_dict(path)
    assert path_dict == expected, "Did not correctly convert path to dictionary"


# ------------------------- TEST POST_PATH -------------------------
@pytest.mark.parametrize(
    "global_path",
    [
        (
            Path(
                waypoints=[
                    HelperLatLon(latitude=48.95, longitude=123.56),
                    HelperLatLon(latitude=38.95, longitude=133.36),
                    HelperLatLon(latitude=28.95, longitude=143.36),
                ]
            )
        )
    ],
)
def test_post_path(global_path: Path):
    """
    Args:
        global_path (Path): The global path to post.
    """

    # Launch http server
    server = ps.run_server()

    assert post_path(global_path, url=ps.POST_TEST_URL), "Failed to post global path"

    ps.shutdown_server(httpd=server)


# ------------------------- TEST WRITE_TO_FILE ------------------------------
@pytest.mark.parametrize(
    "file_path",
    [
        ("/workspaces/sailbot_workspace/src/local_pathfinding/anywhere_else/mock_global_path.csv"),
        (""),
        ("/workspaces/sailbot_workspace/src/local_pathfinding/ global_paths/mock_global_path.csv"),
    ],
)
def test_write_to_file(file_path: str):
    with pytest.raises(ValueError):
        write_to_file(file_path=file_path, global_path=None)
