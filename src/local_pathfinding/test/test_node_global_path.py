import csv
import os
import tempfile
from unittest import mock

import pytest
import yaml
from custom_interfaces.msg import HelperLatLon, Path

import local_pathfinding.node_global_path as ngp
from local_pathfinding.coord_systems import GEODESIC, meters_to_km

_TEST_DIR = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(_TEST_DIR, "..", "..", "global_launch", "config", "globals.yaml")) as f:
    config = yaml.safe_load(f)
GLOBAL_PATH_SPACING_KM = config["/**"]["ros__parameters"]["global_path_interval_spacing_km"]

MOCK_CSV = os.path.join(_TEST_DIR, "..", "global_paths", "mock_global_path.csv")


def _write_csv(rows: list[tuple[float, float]]) -> str:
    """Writes a temporary global path csv with a latitude,longitude header and returns its path."""
    fd, path = tempfile.mkstemp(suffix=".csv")
    with os.fdopen(fd, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["latitude", "longitude"])
        for lat, lon in rows:
            writer.writerow([lat, lon])
    return path


def _max_spacing_km(path: Path) -> float:
    """Returns the largest distance in km between consecutive waypoints in the path."""
    max_km = 0.0
    for i in range(1, len(path.waypoints)):
        dist_m = GEODESIC.inv(
            path.waypoints[i - 1].longitude,
            path.waypoints[i - 1].latitude,
            path.waypoints[i].longitude,
            path.waypoints[i].latitude,
        )[2]
        max_km = max(max_km, meters_to_km(dist_m))
    return max_km


def _mock_get_parameter(values: dict):
    """Returns a stand-in for Node.get_parameter that serves the given parameter values."""

    def _get(name: str):
        value = values[name]
        parameter_value = mock.Mock()
        parameter_value.string_value = value if isinstance(value, str) else ""
        parameter_value.double_value = float(value) if isinstance(value, (int, float)) else 0.0
        parameter = mock.Mock()
        parameter.get_parameter_value.return_value = parameter_value
        return parameter

    return _get


# ------------------------- TEST GET_GLOBAL_PATH: INTERPOLATION -------------------------
@pytest.mark.parametrize(
    "rows",
    [
        # widely-spaced waypoints that require interpolation
        [(48.158391, -130.253906), (48.121368, -137.022955)],
        [(49.0, -123.0), (50.0, -130.0), (48.0, -138.0)],
    ],
)
def test_get_global_path_interpolates_to_spacing(rows):
    file_path = _write_csv(rows)
    try:
        path = ngp.get_global_path(file_path, GLOBAL_PATH_SPACING_KM)

        assert isinstance(path, Path)
        assert len(path.waypoints) >= len(rows), "Interpolation should not drop waypoints"
        for waypoint in path.waypoints:
            assert isinstance(waypoint, HelperLatLon)
            assert isinstance(waypoint.latitude, float)
            assert isinstance(waypoint.longitude, float)
        assert _max_spacing_km(path) <= GLOBAL_PATH_SPACING_KM, "Spacing exceeds interval"
    finally:
        os.remove(file_path)


# ------------------------- TEST GET_GLOBAL_PATH: ENDPOINTS PRESERVED -------------------------
def test_get_global_path_preserves_endpoints():
    rows = [(48.158391, -130.253906), (48.121368, -137.022955)]
    file_path = _write_csv(rows)
    try:
        path = ngp.get_global_path(file_path, GLOBAL_PATH_SPACING_KM)

        assert path.waypoints[0].latitude == pytest.approx(rows[0][0], abs=1e-4)
        assert path.waypoints[0].longitude == pytest.approx(rows[0][1], abs=1e-4)
        assert path.waypoints[-1].latitude == pytest.approx(rows[-1][0], abs=1e-4)
        assert path.waypoints[-1].longitude == pytest.approx(rows[-1][1], abs=1e-4)
    finally:
        os.remove(file_path)


# ------------------------- TEST GET_GLOBAL_PATH: ALREADY WITHIN SPACING -------------------------
def test_get_global_path_close_waypoints_unchanged():
    # two waypoints already within the interval spacing => returned unchanged
    rows = [(48.1000, -130.0000), (48.1100, -130.0000)]  # ~1.1 km apart
    file_path = _write_csv(rows)
    try:
        path = ngp.get_global_path(file_path, GLOBAL_PATH_SPACING_KM)
        assert len(path.waypoints) == len(rows)
    finally:
        os.remove(file_path)


# ------------------------- TEST GET_GLOBAL_PATH: EMPTY CSV -------------------------
def test_get_global_path_empty_csv_returns_empty():
    file_path = _write_csv([])  # header only, no waypoints
    try:
        path = ngp.get_global_path(file_path, GLOBAL_PATH_SPACING_KM)
        assert isinstance(path, Path)
        assert len(path.waypoints) == 0
    finally:
        os.remove(file_path)


# ------------------------- TEST GET_GLOBAL_PATH: REPO CSV -------------------------
def test_get_global_path_reads_repo_csv():
    if not os.path.exists(MOCK_CSV):
        pytest.skip("mock_global_path.csv not found")

    path = ngp.get_global_path(MOCK_CSV, GLOBAL_PATH_SPACING_KM)
    assert isinstance(path, Path)
    assert len(path.waypoints) >= 2
    assert _max_spacing_km(path) <= GLOBAL_PATH_SPACING_KM


# ------------------------- TEST NODE: PUBLISHES A VALID PATH -------------------------
def test_node_publishes_valid_path():
    rows = [(48.158391, -130.253906), (48.121368, -137.022955)]
    file_path = _write_csv(rows)
    try:
        node = ngp.GlobalPath.__new__(ngp.GlobalPath)
        node.global_path = None
        node.file_mtime = None
        node._load_error_logged = False
        node.global_path_pub = mock.Mock()
        node.get_logger = mock.Mock(return_value=mock.Mock())
        node.get_parameter = _mock_get_parameter(
            {
                "global_path_filepath": file_path,
                "global_path_interval_spacing_km": GLOBAL_PATH_SPACING_KM,
            }
        )

        node.publish_global_path()

        assert node.global_path_pub.publish.called, "Node should publish a valid global path"
        published = node.global_path_pub.publish.call_args[0][0]
        assert isinstance(published, Path)
        assert len(published.waypoints) >= 2
    finally:
        os.remove(file_path)


# ------------------------- TEST NODE: SKIPS AN EMPTY PATH -------------------------
def test_node_skips_empty_path():
    file_path = _write_csv([])  # header only, no waypoints
    try:
        node = ngp.GlobalPath.__new__(ngp.GlobalPath)
        node.global_path = None
        node.file_mtime = None
        node._load_error_logged = False
        node.global_path_pub = mock.Mock()
        node.get_logger = mock.Mock(return_value=mock.Mock())
        node.get_parameter = _mock_get_parameter(
            {
                "global_path_filepath": file_path,
                "global_path_interval_spacing_km": GLOBAL_PATH_SPACING_KM,
            }
        )

        node.publish_global_path()

        assert not node.global_path_pub.publish.called, "Empty path must not be published"
    finally:
        os.remove(file_path)
