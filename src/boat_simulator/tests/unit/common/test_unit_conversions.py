"""Tests classes and functions in boat_simulator/common/unit_conversions.py"""

import math
import numpy as np

import pytest

from boat_simulator.common.unit_conversions import (
    ConversionFactor,
    ConversionFactors,
    UnitConverter,
)


class TestConversionFactor:
    @pytest.mark.parametrize(
        "factor, initial_value, expected_converted_value",
        [(60, 1, 60), (0.5, 2, 0.5 * 2), (-10.2, 14.22, -10.2 * 14.22)],
    )
    def test_forward_convert(self, factor, initial_value, expected_converted_value):
        conversion_factor = ConversionFactor(factor=factor)
        actual_converted_value = conversion_factor.forward_convert(initial_value)
        assert math.isclose(actual_converted_value, expected_converted_value)

    @pytest.mark.parametrize(
        "factor, initial_value, expected_converted_value",
        [(60, 60, 1), (0.5, 1, 1 / 0.5), (-10.2, -10.2 * 14.22, 14.22)],
    )
    def test_backward_convert(self, factor, initial_value, expected_converted_value):
        conversion_factor = ConversionFactor(factor=factor)
        actual_converted_value = conversion_factor.backward_convert(initial_value)
        assert math.isclose(actual_converted_value, expected_converted_value)

    @pytest.mark.parametrize("factor", [1, 2, 5, 10, 0.5, -18.9, -17, -13.33])
    def test_inverse(self, factor):
        conversion_factor = ConversionFactor(factor=factor)
        inverse_conversion_factor = conversion_factor.inverse()
        assert math.isclose(inverse_conversion_factor.factor, 1 / factor)
        assert math.isclose(inverse_conversion_factor.factor, conversion_factor.inverse_factor)

    @pytest.mark.parametrize(
        "factor1, factor2, expected_product_factor",
        [(1, 1, 1), (2, 5, 10), (1 / 2, 2, 1), (0, 1, 0)],
    )
    def test_multiplication(self, factor1, factor2, expected_product_factor):
        conversion_factor1 = ConversionFactor(factor=factor1)
        conversion_factor2 = ConversionFactor(factor=factor2)
        product_conversion_factor = conversion_factor1 * conversion_factor2
        reverse_product_conversion_factor = conversion_factor2 * conversion_factor1
        assert math.isclose(product_conversion_factor.factor, expected_product_factor)
        assert math.isclose(reverse_product_conversion_factor.factor, expected_product_factor)
        assert math.isclose(
            product_conversion_factor.factor, reverse_product_conversion_factor.factor
        )


class TestUnitConverter:
    def test_init(self):
        """
        Test instance attribute assignment using attributes or using kwargs into constructor
        """
        unit_converter1 = UnitConverter(
            prop1=ConversionFactors.sec_to_min, prop2=ConversionFactors.sec_to_h
        )
        assert unit_converter1.prop1 == ConversionFactors.sec_to_min
        assert unit_converter1.prop2 == ConversionFactors.sec_to_h

        converted_values1 = unit_converter1.convert(prop1=120, prop2=3600)

        assert converted_values1["prop1"] == 2.0
        assert converted_values1["prop2"] == 1.0

        conversion_factors = {
            "prop1": ConversionFactors.sec_to_min,
            "prop2": ConversionFactors.sec_to_h,
        }
        values = {"prop1": 120, "prop2": 3600}
        unit_converter2 = UnitConverter(**conversion_factors)

        assert unit_converter2.prop1 == ConversionFactors.sec_to_min
        assert unit_converter2.prop2 == ConversionFactors.sec_to_h

        converted_values2 = unit_converter2.convert(**values)

        assert converted_values2["prop1"] == 2.0
        assert converted_values2["prop2"] == 1.0

    def test_convert_m_km(self):
        unit_converter = UnitConverter(
            m_to_km=ConversionFactors.m_to_km, km_to_m=ConversionFactors.km_to_m
        )

        converted_values = unit_converter.convert(m_to_km=1000, km_to_m=1)

        assert converted_values["m_to_km"] == 1
        assert converted_values["km_to_m"] == 1000

    def test_convert_cm_m(self):
        unit_converter = UnitConverter(
            cm_to_m=ConversionFactors.cm_to_m, m_to_cm=ConversionFactors.m_to_cm
        )

        converted_values = unit_converter.convert(cm_to_m=100, m_to_cm=1)

        assert converted_values["cm_to_m"] == 1
        assert converted_values["m_to_cm"] == 100

    def test_convert_cm_km(self):
        unit_converter = UnitConverter(
            km_to_cm=ConversionFactors.km_to_cm, cm_to_km=ConversionFactors.cm_to_km
        )

        converted_values = unit_converter.convert(km_to_cm=1, cm_to_km=1e5)

        assert converted_values["km_to_cm"] == 1e5
        assert converted_values["cm_to_km"] == 1

    def test_convert_ft_m(self):
        unit_converter = UnitConverter(
            m_to_ft=ConversionFactors.m_to_ft, ft_to_m=ConversionFactors.ft_to_m
        )

        converted_values = unit_converter.convert(m_to_ft=100.0, ft_to_m=10)

        assert math.isclose(converted_values["m_to_ft"], 328.084, abs_tol=1e-6)
        assert math.isclose(converted_values["ft_to_m"], 3.048, abs_tol=1e-6)

    def test_convert_ft_mi(self):
        unit_converter = UnitConverter(
            mi_to_ft=ConversionFactors.mi_to_ft, ft_to_mi=ConversionFactors.ft_to_mi
        )

        converted_values = unit_converter.convert(mi_to_ft=1, ft_to_mi=1000)

        assert converted_values["mi_to_ft"] == 5280
        assert math.isclose(converted_values["ft_to_mi"], 0.189394, abs_tol=1e-6)

    def test_convert_m_mi(self):
        unit_converter = UnitConverter(
            mi_to_m=ConversionFactors.mi_to_m, m_to_mi=ConversionFactors.m_to_mi
        )

        converted_values = unit_converter.convert(mi_to_m=10, m_to_mi=1e4)

        assert math.isclose(converted_values["mi_to_m"], 16093.44, abs_tol=1e-6)
        assert math.isclose(converted_values["m_to_mi"], 6.2137119, abs_tol=1e-6)

    def test_convert_km_mi(self):
        unit_converter = UnitConverter(
            km_to_mi=ConversionFactors.km_to_mi, mi_to_km=ConversionFactors.mi_to_km
        )

        converted_values = unit_converter.convert(km_to_mi=1.0, mi_to_km=10.0)

        assert math.isclose(converted_values["km_to_mi"], 0.62137119, abs_tol=1e-6)
        assert converted_values["mi_to_km"] == 16.09344

    def test_convert_mi_nautical_mi(self):
        unit_converter = UnitConverter(
            nat_mi_to_mi=ConversionFactors.nautical_mi_to_mi,
            mi_to_nat_mi=ConversionFactors.mi_to_nautical_mi,
        )

        converted_values = unit_converter.convert(nat_mi_to_mi=1.0, mi_to_nat_mi=1.0)

        assert converted_values["nat_mi_to_mi"] == 1.15078
        assert math.isclose(converted_values["mi_to_nat_mi"], 0.868976, abs_tol=1e-6)

    def test_convert_km_nautical_mi(self):
        unit_converter = UnitConverter(
            nat_mi_to_km=ConversionFactors.nautical_mi_to_km,
            km_to_nat_mi=ConversionFactors.km_to_nautical_mi,
        )

        converted_values = unit_converter.convert(nat_mi_to_km=1.0, km_to_nat_mi=1.0)

        assert converted_values["nat_mi_to_km"] == 1.852
        assert math.isclose(converted_values["km_to_nat_mi"], 0.539957, abs_tol=1e-6)

    def test_convert_sec_min(self):
        unit_converter = UnitConverter(
            sec_to_min=ConversionFactors.sec_to_min,
            min_to_sec=ConversionFactors.min_to_sec,
        )

        converted_values = unit_converter.convert(sec_to_min=120.0, min_to_sec=10.0)

        assert converted_values["sec_to_min"] == 2.0
        assert converted_values["min_to_sec"] == 600.0

    def test_convert_sec_h(self):
        unit_converter = UnitConverter(
            sec_to_h=ConversionFactors.sec_to_h,
            h_to_sec=ConversionFactors.h_to_sec,
        )

        converted_values = unit_converter.convert(sec_to_h=3600, h_to_sec=1)

        assert converted_values["sec_to_h"] == 1
        assert converted_values["h_to_sec"] == 3600

    def test_convert_min_h(self):
        unit_converter = UnitConverter(
            min_to_h=ConversionFactors.min_to_h,
            h_to_min=ConversionFactors.h_to_min,
        )

        converted_values = unit_converter.convert(min_to_h=90, h_to_min=2)

        assert converted_values["min_to_h"] == 1.5
        assert converted_values["h_to_min"] == 120

    def test_convert_miPh_kmPh(self):
        unit_converter = UnitConverter(
            miPh_to_kmPh=ConversionFactors.miPh_to_kmPh,
            kmPh_to_miPh=ConversionFactors.kmPh_to_miPh,
        )

        converted_values = unit_converter.convert(miPh_to_kmPh=10.0, kmPh_to_miPh=10.0)

        assert math.isclose(converted_values["miPh_to_kmPh"], 16.09344, abs_tol=1e-6)
        assert math.isclose(converted_values["kmPh_to_miPh"], 6.2137119, abs_tol=1e-6)

    def test_convert_mPs_kmPh(self):
        unit_converter = UnitConverter(
            mPs_to_kmPh=ConversionFactors.mPs_to_kmPh,
            kmPh_to_mPs=ConversionFactors.kmPh_to_mPs,
        )

        converted_values = unit_converter.convert(mPs_to_kmPh=1.0, kmPh_to_mPs=1.0)

        assert converted_values["mPs_to_kmPh"] == 3.6
        assert math.isclose(converted_values["kmPh_to_mPs"], 0.277778, abs_tol=1e-6)

    def test_convert_knots_kmPh(self):
        unit_converter = UnitConverter(
            knots_to_kmPh=ConversionFactors.knots_to_kmPh,
            kmPh_to_knots=ConversionFactors.kmPh_to_knots,
        )

        converted_values = unit_converter.convert(knots_to_kmPh=1.0, kmPh_to_knots=1.0)

        assert math.isclose(converted_values["knots_to_kmPh"], 1.852, abs_tol=1e-6)
        assert math.isclose(converted_values["kmPh_to_knots"], 0.539957, abs_tol=1e-6)

    def test_convert_knots_miPh(self):
        unit_converter = UnitConverter(
            knots_to_miPh=ConversionFactors.knots_to_miPh,
            miPh_to_knots=ConversionFactors.miPh_to_knots,
        )

        converted_values = unit_converter.convert(knots_to_miPh=1.0, miPh_to_knots=1.0)

        assert math.isclose(converted_values["knots_to_miPh"], 1.15078, abs_tol=1e-6)
        assert math.isclose(converted_values["miPh_to_knots"], 0.868976, abs_tol=1e-6)

    def test_convert_miPs2_mPs2(self):
        unit_converter = UnitConverter(
            miPs2_to_mPs2=ConversionFactors.miPs2_to_mPs2,
            mPs2_to_miPs2=ConversionFactors.mPs2_to_miPs2,
        )

        converted_values = unit_converter.convert(miPs2_to_mPs2=1.0, mPs2_to_miPs2=1.0)

        assert math.isclose(converted_values["miPs2_to_mPs2"], 1609.344, abs_tol=1e-6)
        assert math.isclose(converted_values["mPs2_to_miPs2"], 0.000621371192, abs_tol=1e-6)

    def test_convert_kmPs2_mPs2(self):
        unit_converter = UnitConverter(
            kmPs2_to_mPs2=ConversionFactors.kmPs2_to_mPs2,
            mPs2_to_kmPs2=ConversionFactors.mPs2_to_kmPs2,
        )

        converted_values = unit_converter.convert(kmPs2_to_mPs2=1.0, mPs2_to_kmPs2=1.0)

        assert converted_values["kmPs2_to_mPs2"] == 1e3
        assert converted_values["mPs2_to_kmPs2"] == 1e-3

    def test_convert_mPs2_knotsPs2(self):
        unit_converter = UnitConverter(
            mPs2_to_knotsPs2=ConversionFactors.mPs2_to_knotsPs2,
            knotsPs2_to_mPs2=ConversionFactors.knotsPs2_to_mPs2,
        )

        converted_values = unit_converter.convert(mPs2_to_knotsPs2=1.0, knotsPs2_to_mPs2=1.0)

        assert math.isclose(converted_values["mPs2_to_knotsPs2"], 1.94384466, abs_tol=1e-6)
        assert math.isclose(converted_values["knotsPs2_to_mPs2"], 0.514444, abs_tol=1e-6)

    def test_convert_kg_g(self):
        unit_convertor = UnitConverter(
            kg_to_g=ConversionFactors.kg_to_g, g_to_kg=ConversionFactors.g_to_kg
        )

        converted_values = unit_convertor.convert(kg_to_g=1.0, g_to_kg=1000.0)

        assert converted_values["kg_to_g"] == 1000.0
        assert converted_values["g_to_kg"] == 1.0

    def test_convert_lb_g(self):
        unit_convertor = UnitConverter(
            lb_to_g=ConversionFactors.lb_to_g, g_to_lb=ConversionFactors.g_to_lb
        )

        converted_values = unit_convertor.convert(lb_to_g=2.5, g_to_lb=1.0)

        assert math.isclose(converted_values["lb_to_g"], 1133.980925, abs_tol=1e-6)
        assert math.isclose(converted_values["g_to_lb"], 0.00220462, abs_tol=1e-6)

    def test_convert_kg_lb(self):
        unit_convertor = UnitConverter(
            kg_to_lb=ConversionFactors.kg_to_lb, lb_to_kg=ConversionFactors.lb_to_kg
        )

        converted_values = unit_convertor.convert(kg_to_lb=1.5, lb_to_kg=1.0)

        assert math.isclose(converted_values["kg_to_lb"], 3.3069339328, abs_tol=1e-6)
        assert math.isclose(converted_values["lb_to_kg"], 0.45359237, abs_tol=1e-6)

    @pytest.mark.parametrize(
        "degrees_to_rad, expected_result",
        [(0, 0), (90, math.pi / 2), (180, math.pi), (360, 2 * math.pi), (-180, -math.pi)],
    )
    def test_convert_degrees_to_rad(self, degrees_to_rad, expected_result):
        unit_convertor = UnitConverter(
            degrees_to_rad=ConversionFactors.degrees_to_rad,
        )

        converted_values = unit_convertor.convert(degrees_to_rad=degrees_to_rad)

        assert math.isclose(converted_values["degrees_to_rad"], expected_result, abs_tol=1e-6)

    @pytest.mark.parametrize(
        "rad_to_degrees, expected_result",
        [(0, 0), (math.pi / 2, 90), (math.pi, 180), (2 * math.pi, 360), (-math.pi, -180)],
    )
    def test_convert_rad_to_degrees(self, rad_to_degrees, expected_result):
        unit_convertor = UnitConverter(
            rad_to_degrees=ConversionFactors.rad_to_degrees,
        )

        converted_values = unit_convertor.convert(rad_to_degrees=rad_to_degrees)

        assert math.isclose(converted_values["rad_to_degrees"], expected_result, abs_tol=1e-6)

    @pytest.mark.parametrize(
        "km_to_m, expected_result",
        [
            ([0, 0, 0], [0, 0, 0]),
            ([0, 1, 2], [0, 1000, 2000]),
            ([], []),
            ([1], [1000]),
        ],
    )
    def test_convert_arraylike(self, km_to_m, expected_result):
        unit_convertor = UnitConverter(
            km_to_m=ConversionFactors.km_to_m,
        )

        converted_values = unit_convertor.convert(km_to_m=np.array(km_to_m))

        assert np.array_equal(converted_values["km_to_m"], np.array(expected_result))
