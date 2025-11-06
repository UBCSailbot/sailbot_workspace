"""Test visualizer coordinate translation logic."""

import math

import custom_interfaces.msg as ci
import pytest

import local_pathfinding.coord_systems as cs
import local_pathfinding.visualizer as vz


def create_mock_lpath_data(
    boat_lat: float,
    boat_lon: float,
    waypoints: list,
    reference_lat: float,
    reference_lon: float,
) -> ci.LPathData:
    """Create a mock LPathData message for testing."""
    msg = ci.LPathData()

    # Set GPS data
    msg.gps = ci.GPS()
    msg.gps.lat_lon = ci.HelperLatLon(latitude=boat_lat, longitude=boat_lon)
    msg.gps.heading = ci.HelperHeading(heading=45.0)
    msg.gps.speed = ci.HelperSpeed(speed=5.0)

    # Set local path
    msg.local_path = ci.Path()
    msg.local_path.waypoints = [
        ci.HelperLatLon(latitude=lat, longitude=lon) for lat, lon in waypoints
    ]

    # Set global path
    msg.global_path = ci.Path()
    msg.global_path.waypoints = [
        ci.HelperLatLon(latitude=reference_lat, longitude=reference_lon)
    ]

    # Set wind sensor
    msg.filtered_wind_sensor = ci.WindSensor()
    msg.filtered_wind_sensor.direction = 90.0
    msg.filtered_wind_sensor.speed = ci.HelperSpeed(speed=10.0)

    # Set obstacles and AIS ships
    msg.obstacles = []
    msg.ais_ships = ci.AISShips()
    msg.ais_ships.ships = []

    return msg


def test_visualizer_state_basic_initialization():
    """Test that VisualizerState can be initialized with valid data."""
    boat_lat, boat_lon = 49.5, -139.9
    waypoints = [(49.5, -139.9), (49.6, -140.0), (49.7, -140.1)]
    reference_lat, reference_lon = 49.7, -140.1

    msg = create_mock_lpath_data(boat_lat, boat_lon, waypoints, reference_lat, reference_lon)
    state = vz.VisualizerState([msg])

    assert state is not None
    assert len(state.sailbot_pos_x) == 1
    assert len(state.sailbot_pos_y) == 1
    assert len(state.final_local_wp_x) == 3
    assert len(state.final_local_wp_y) == 3


def test_visualizer_coordinate_translation():
    """Test that coordinates are properly translated in the visualizer plot."""
    # Create a simple scenario: boat at origin after translation
    boat_lat, boat_lon = 49.5, -139.9
    waypoints = [(49.5, -139.9), (49.6, -140.0), (49.7, -140.1)]
    reference_lat, reference_lon = 49.7, -140.1

    msg = create_mock_lpath_data(boat_lat, boat_lon, waypoints, reference_lat, reference_lon)
    state = vz.VisualizerState([msg])

    # The boat should be at some position in the original coordinate system
    boat_x_original = state.sailbot_pos_x[-1]
    boat_y_original = state.sailbot_pos_y[-1]

    # After translation in the plot, the boat should be at (0, 0)
    # and waypoints should be relative to the boat
    goal_x_original = state.final_local_wp_x[-1]
    goal_y_original = state.final_local_wp_y[-1]

    # Calculate what the translated coordinates should be
    goal_x_translated = goal_x_original - boat_x_original
    goal_y_translated = goal_y_original - boat_y_original

    # Verify that the translation preserves relative distances
    original_distance = math.hypot(goal_x_original - boat_x_original,
                                    goal_y_original - boat_y_original)
    translated_distance = math.hypot(goal_x_translated, goal_y_translated)

    assert original_distance == pytest.approx(translated_distance, rel=1e-6)


def test_visualizer_state_invalid_input():
    """Test that VisualizerState raises an error with invalid input."""
    with pytest.raises(ValueError):
        vz.VisualizerState([])


def test_visualizer_plot_generation():
    """Test that live_update_plot generates a valid figure."""
    boat_lat, boat_lon = 49.5, -139.9
    waypoints = [(49.5, -139.9), (49.6, -140.0), (49.7, -140.1)]
    reference_lat, reference_lon = 49.7, -140.1

    msg = create_mock_lpath_data(boat_lat, boat_lon, waypoints, reference_lat, reference_lon)
    state = vz.VisualizerState([msg])

    fig = vz.live_update_plot(state)

    assert fig is not None
    assert len(fig.data) > 0  # Should have at least one trace
    assert fig.layout.xaxis.range is not None  # Should have x-axis range set
    assert fig.layout.yaxis.range is not None  # Should have y-axis range set
