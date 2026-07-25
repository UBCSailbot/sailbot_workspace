import pytest

from controller.common.unit_conversions import kmph_to_mps


@pytest.mark.parametrize(
    "speed_kmph, expected_speed_mps",
    [
        (0.0, 0.0),
        (3.6, 1.0),
        (36.0, 10.0),
    ],
)
def test_kmph_to_mps(speed_kmph: float, expected_speed_mps: float):
    assert kmph_to_mps(speed_kmph) == pytest.approx(expected_speed_mps)
