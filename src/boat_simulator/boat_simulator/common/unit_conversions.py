"""Unit conversion logic, mostly contained in the `UnitConverter` class."""

from __future__ import annotations

import math
from enum import Enum
from typing import Dict

from boat_simulator.common.types import EnumAttr, Scalar, ScalarOrArray


class ConversionFactor:
    """Performs unit conversions from one unit to another. Both directions of unit conversion are
    supported by this class.

    Attributes:
        `factor` (Scalar): The conversion factor to go from unit A to B.
        `inverse_factor` (Scalar): The conversion factor to go from unit B to A.
    """

    def __init__(self, factor: Scalar):
        """Initializes an instance of `ConversionFactor`.

        Args:
            factor (Scalar): Conversion factor from unit A to B.
        """
        self.__factor = factor

    def forward_convert(self, value: ScalarOrArray) -> ScalarOrArray:
        """Convert from unit A to B.

        Args:
            value (ScalarOrArray): Values with unit A to be converted.

        Returns:
            ScalarOrArray: Converted values with unit B.
        """
        return value * self.factor

    def backward_convert(self, value: ScalarOrArray) -> ScalarOrArray:
        """Convert from unit B to A.

        Args:
            value (ScalarOrArray): Values with unit B to be converted.

        Returns:
            ScalarOrArray: Converted values with unit A.
        """
        return value * self.inverse_factor

    def inverse(self) -> ConversionFactor:
        """
        Get the inverse of this class containing the inverse conversion factor.

        Returns:
            ConversionFactor: The inverse of this class.
        """
        return ConversionFactor(factor=self.inverse_factor)

    def __mul__(self, other: ConversionFactor) -> ConversionFactor:
        """Multiplication operator between two `ConversionFactor` objects (A * B).

        Args:
            other (ConversionFactor): Other conversion factor being multiplied.

        Returns:
            ConversionFactor: Multiplied conversion factor.
        """
        mul_conversion_factor = self.factor * other.factor
        return ConversionFactor(factor=mul_conversion_factor)

    def __rmul__(self, other: ConversionFactor) -> ConversionFactor:
        """Multiplication operator between two `ConversionFactor` objects (B * A).

        Args:
            other (ConversionFactor): Other conversion factor being multiplied.

        Returns:
            ConversionFactor: Multiplied conversion factor.
        """
        return self.__mul__(other)

    @property
    def factor(self) -> Scalar:
        return self.__factor

    @property
    def inverse_factor(self) -> Scalar:
        return 1 / self.factor


class ConversionFactors(Enum):
    """Predefined conversion factors commonly used in that boat simulator. This class is meant
    to be used in conjunction with the `UnitConverter` class to specify the unit conversions
    that will be performed.

    Attributes:
        <UNIT A>_to_<UNIT B> (EnumAttr): `ConversionFactor` classes to perform unit
            conversions going from unit A to B.

        Attributes in this class must follow the above naming convention.
    """

    # Length

    km_to_m = ConversionFactor(factor=1000)
    m_to_km = km_to_m.inverse()

    m_to_cm = ConversionFactor(factor=100)
    cm_to_m = m_to_cm.inverse()

    km_to_cm = km_to_m * m_to_cm
    cm_to_km = km_to_cm.inverse()

    m_to_ft = ConversionFactor(factor=3.28084)
    ft_to_m = m_to_ft.inverse()

    mi_to_ft = ConversionFactor(factor=5280)
    ft_to_mi = mi_to_ft.inverse()

    mi_to_m = ConversionFactor(factor=1609.344)
    m_to_mi = mi_to_m.inverse()

    mi_to_km = mi_to_m * m_to_km
    km_to_mi = mi_to_km.inverse()

    nautical_mi_to_mi = ConversionFactor(factor=1.15078)
    mi_to_nautical_mi = nautical_mi_to_mi.inverse()

    nautical_mi_to_km = ConversionFactor(factor=1.852)
    km_to_nautical_mi = nautical_mi_to_km.inverse()

    # Time
    min_to_sec = ConversionFactor(factor=60)
    sec_to_min = min_to_sec.inverse()

    h_to_min = ConversionFactor(factor=60)
    min_to_h = h_to_min.inverse()

    h_to_sec = h_to_min * min_to_sec
    sec_to_h = h_to_sec.inverse()

    # Speed
    miPh_to_kmPh = ConversionFactor(factor=1.609344)
    kmPh_to_miPh = miPh_to_kmPh.inverse()

    mPs_to_kmPh = ConversionFactor(factor=3.6)
    kmPh_to_mPs = mPs_to_kmPh.inverse()

    knots_to_kmPh = ConversionFactor(factor=1.852)
    kmPh_to_knots = knots_to_kmPh.inverse()

    knots_to_miPh = ConversionFactor(factor=1.15077945)
    miPh_to_knots = knots_to_miPh.inverse()

    # Acceleration

    miPs2_to_mPs2 = mi_to_m
    mPs2_to_miPs2 = m_to_mi

    kmPs2_to_mPs2 = km_to_m
    mPs2_to_kmPs2 = m_to_km

    mPs2_to_knotsPs2 = ConversionFactor(factor=1.94384466)
    knotsPs2_to_mPs2 = mPs2_to_knotsPs2.inverse()

    # Mass

    kg_to_g = ConversionFactor(factor=1000)
    g_to_kg = kg_to_g.inverse()

    lb_to_g = ConversionFactor(factor=453.59237)
    g_to_lb = lb_to_g.inverse()

    kg_to_lb = ConversionFactor(factor=2.2046226218)
    lb_to_kg = kg_to_lb.inverse()

    # Rotation

    degrees_to_rad = ConversionFactor(factor=math.pi / 180)
    rad_to_degrees = degrees_to_rad.inverse()


class UnitConverter:
    """Performs multiple unit conversions at once.

    Attributes:
        <ATTR NAME> (EnumAttr): Attribute names of this class depend on what is passed into
            the __init__ function of this class. All attributes are of type `EnumAttr`, which
            should come from the `ConversionFactors` class.
    """

    def __init__(self, **kwargs: EnumAttr):
        """Initializes an instance of `UnitConverter`.

        Args:
            kwargs (Dict[str, EnumAttr]): Dictionary keys are class attribute names, and dictionary
                values are class attribute values. Dictionary values are strictly class attributes
                belonging to `ConversionFactors`.
        """
        for attr_name, attr_val in kwargs.items():
            assert isinstance(attr_val, Enum) and isinstance(attr_val.value, ConversionFactor)
            setattr(self, attr_name, attr_val)

    def convert(self, **kwargs: ScalarOrArray) -> Dict[str, ScalarOrArray]:
        """Perform unit conversions for multiple specified values.

        Pre-Condition:
            Unit conversions are only done on comparable ScalarOrArray. Ex: Length to length

        Args:
            kwargs (Dict[str, ScalarOrArray]): Dictionary keys are strictly names
            of attributes belonging to this class.
            Dictionary values are the values to be converted, using the
            conversion factor corresponding to the class attribute.

        Returns:
            Dict[str, ScalarOrArray]: Converted values. Dictionary keys are class
            attribute names corresponding to the converted value. Dictionary values are the
            converted values.
        """
        converted_values: Dict[str, ScalarOrArray] = {}

        for attr_name, attr_val in kwargs.items():
            attr = getattr(self, attr_name, None)
            assert attr is not None, f"Attribute name {attr} not found in UnitConverter."

            conversion_factor = attr.value
            converted_values[attr_name] = conversion_factor.forward_convert(attr_val)

        return converted_values
