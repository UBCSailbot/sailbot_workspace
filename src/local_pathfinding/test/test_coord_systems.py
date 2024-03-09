import math

import pytest
from custom_interfaces.msg import HelperLatLon

import local_pathfinding.coord_systems as coord_systems


@pytest.mark.parametrize(
    "cartesian,true_bearing",
    [
        (0.0, 90.0),
        (90.0, 0.0),
        (180.0, 270.0),
        (270.0, 180.0),
    ],
)
def test_cartesian_to_true_bearing(cartesian: float, true_bearing: float):
    assert coord_systems.cartesian_to_true_bearing(cartesian) == pytest.approx(
        true_bearing
    ), "incorrect angle conversion"


@pytest.mark.parametrize(
    "meters,km",
    [(0.0, 0.0), (30, 0.03), (500, 0.5), (-30.5, -0.0305), (-0.0, 0.0)],
)
def test_meters_to_km(meters: float, km: float):
    assert coord_systems.meters_to_km(meters) == pytest.approx(km), "incorrect distance conversion"


@pytest.mark.parametrize(
    "km,meters",
    [(0.0, 0.0), (0.03, 30), (0.5, 500), (-0.0305, -30.5), (-0.0, 0.0)],
)
def test_km_to_meters(km: float, meters: float):
    assert coord_systems.km_to_meters(km) == pytest.approx(meters), "incorrect distance conversion"


@pytest.mark.parametrize(
    "ref_lat,ref_lon,true_bearing_deg,dist_km",
    [
        (30.0, -123.0, 0.00, 30.0),
        (30.0, -123.0, 45.0, 30.0),
        (30.0, -123.0, 90.0, 30.0),
        (60.0, -123.0, 0.00, 30.0),
        (60.0, -123.0, 45.0, 30.0),
        (60.0, -123.0, 90.0, 30.0),
    ],
)
def test_latlon_to_xy(ref_lat: float, ref_lon: float, true_bearing_deg: float, dist_km: float):
    # create inputs
    reference = HelperLatLon(latitude=ref_lat, longitude=ref_lon)
    lon, lat, _ = coord_systems.GEODESIC.fwd(
        lons=ref_lon, lats=ref_lat, az=true_bearing_deg, dist=dist_km * 1000
    )
    latlon = HelperLatLon(latitude=lat, longitude=lon)

    # create expected output
    true_bearing = math.radians(true_bearing_deg)
    xy = coord_systems.XY(
        x=dist_km * math.sin(true_bearing),
        y=dist_km * math.cos(true_bearing),
    )

    assert coord_systems.latlon_to_xy(reference, latlon) == pytest.approx(
        xy
    ), "incorrect coordinate conversion"


@pytest.mark.parametrize(
    "ref_lat,ref_lon,true_bearing_deg,dist_km",
    [
        (30.0, -123.0, 0.00, 30.0),
        (30.0, -123.0, 45.0, 30.0),
        (30.0, -123.0, 90.0, 30.0),
        (60.0, -123.0, 0.00, 30.0),
        (60.0, -123.0, 45.0, 30.0),
        (60.0, -123.0, 90.0, 30.0),
        (60.0, -123.0, -120.0, 30.0),
    ],
)
def test_xy_to_latlon(ref_lat: float, ref_lon: float, true_bearing_deg: float, dist_km: float):
    # create inputs
    true_bearing = math.radians(true_bearing_deg)
    xy = coord_systems.XY(
        x=dist_km * math.sin(true_bearing),
        y=dist_km * math.cos(true_bearing),
    )

    # create expected output
    reference = HelperLatLon(latitude=ref_lat, longitude=ref_lon)
    lon, lat, _ = coord_systems.GEODESIC.fwd(
        lons=ref_lon, lats=ref_lat, az=true_bearing_deg, dist=dist_km * 1000
    )
    latlon = HelperLatLon(latitude=lat, longitude=lon)

    converted_latlon = coord_systems.xy_to_latlon(reference, xy)
    assert (converted_latlon.latitude, converted_latlon.longitude) == pytest.approx(
        (latlon.latitude, latlon.longitude)
    ), "incorrect coordinate conversion"
