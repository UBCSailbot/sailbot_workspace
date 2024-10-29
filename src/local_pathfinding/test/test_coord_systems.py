import math
from typing import List

import pytest
from custom_interfaces.msg import HelperLatLon
from shapely.geometry import MultiPolygon, Polygon, box

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


# Test latlon_polygon_list_to_xy_polygon_list
# just asserts that every point in every xy_polygon agrees with latlon_to_xy() from coord_systems
@pytest.mark.parametrize(
    "latlon_polygons, reference_point",
    [
        (
            list(
                [
                    Polygon(
                        [
                            (-129.10434, 49.173085),
                            (-131.23681, 50.112124),
                            (-134.820239, 50.658515),
                            (-135.963419, 49.772751),
                            (-136.359135, 48.230528),
                            (-134.556428, 47.671306),
                            (-131.478636, 47.78954),
                            (-129.895772, 48.274419),
                            (-129.412119, 48.928274),
                        ]
                    ),
                    Polygon(
                        [
                            (-123.872094, 50.252825),
                            (-124.135905, 49.530913),
                            (-125.938612, 49.758558),
                            (-125.674801, 50.797603),
                        ]
                    ),
                    Polygon(
                        [
                            (-123.872094, 50.252825),
                            (-124.135905, 49.530913),
                            (-125.938612, 49.758558),
                            (-125.674801, 50.797603),
                        ]
                    ),
                ]
            ),
            HelperLatLon(latitude=51.527884, longitude=-132.643800),
        ),
        (
            MultiPolygon(
                [
                    Polygon(
                        [
                            (-129.10434, 49.173085),
                            (-131.23681, 50.112124),
                            (-134.820239, 50.658515),
                            (-135.963419, 49.772751),
                            (-136.359135, 48.230528),
                            (-134.556428, 47.671306),
                            (-131.478636, 47.78954),
                            (-129.895772, 48.274419),
                            (-129.412119, 48.928274),
                        ]
                    ),
                    Polygon(
                        [
                            (-123.872094, 50.252825),
                            (-124.135905, 49.530913),
                            (-125.938612, 49.758558),
                            (-125.674801, 50.797603),
                        ]
                    ),
                    Polygon(
                        [
                            (-123.872094, 50.252825),
                            (-124.135905, 49.530913),
                            (-125.938612, 49.758558),
                            (-125.674801, 50.797603),
                        ]
                    ),
                ]
            ).geoms,
            HelperLatLon(latitude=51.527884, longitude=-132.643800),
        ),
        (
            list(
                [
                    Polygon(
                        [
                            (-123.872094, 50.252825),
                            (-124.135905, 49.530913),
                            (-125.938612, 49.758558),
                            (-125.674801, 50.797603),
                        ]
                    ),
                ]
            ),
            HelperLatLon(latitude=51.527884, longitude=-132.643800),
        ),
        (
            list([]),
            HelperLatLon(latitude=51.527884, longitude=-132.643800),
        ),
    ],
)
def test_latlon_polygons_to_xy_polygons(
    latlon_polygons: List[Polygon], reference_point: HelperLatLon
):

    xy_polygons = coord_systems.latlon_polygon_list_to_xy_polygon_list(
        latlon_polygons, reference_point
    )
    assert isinstance(xy_polygons, list)
    assert len(xy_polygons) == len(latlon_polygons)

    if len(xy_polygons) > 0:
        for i, xy_poly in enumerate(xy_polygons):
            latlon_poly = latlon_polygons[i]
            assert isinstance(xy_poly, Polygon)
            assert xy_poly.exterior.coords is not None

            for j, xy_point in enumerate(xy_poly.exterior.coords):
                latlon_point = latlon_poly.exterior.coords[j]
                assert isinstance(xy_point, tuple)
                assert xy_point == pytest.approx(
                    coord_systems.latlon_to_xy(
                        reference_point,
                        HelperLatLon(longitude=latlon_point[0], latitude=latlon_point[1]),
                    )
                )


def test_latlon_polygons_to_xy_polygons_empty_Polygon():

    reference_point = HelperLatLon(latitude=51.527884, longitude=-132.643800)
    b1 = box(0, 0, 1, 1)
    b3 = box(2, 2, 3, 3)

    empty_poly = b1.intersection(b3)
    assert isinstance(empty_poly, Polygon)
    assert empty_poly.is_empty

    result = coord_systems.latlon_polygon_list_to_xy_polygon_list([empty_poly], reference_point)
    assert isinstance(result, List)
    assert len(result) == 1
    assert result[0].is_empty
