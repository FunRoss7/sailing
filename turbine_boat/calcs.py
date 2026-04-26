#!/usr/bin/env python3
from scipy.optimize import minimize_scalar, brentq
import numpy as np
import pint
import math

ureg = pint.UnitRegistry()


class WindTurbine:
    """
    Axial-flow wind turbine modeled with actuator disk theory.
    Operates in air; geometry and aero coefficients computed at construction.
    """

    def __init__(self, diameter, optimal_lambda=2 * math.pi, axial_induction=1/3):
        self.diameter = diameter
        self.optimal_lambda = optimal_lambda
        self.axial_induction = axial_induction

        # Derived geometry
        self.radius = diameter / 2
        self.area = math.pi * self.radius ** 2
        self.pitch_to_diameter = math.pi / optimal_lambda

        # Aero coefficients (actuator disk)
        a = axial_induction
        self.ct = 4 * a * (1 - a)
        self.cp = 4 * a * (1 - a) ** 2

        # Set at solve time
        self.apparent_wind_speed = None
        self.antithrust = None
        self.power = None
        self.shaft_speed_hz = None

    def solve(self, apparent_wind_speed, air_density=1.225 * ureg.kg / ureg.meter ** 3):
        """Compute turbine outputs for a given apparent wind speed."""
        self.apparent_wind_speed = apparent_wind_speed

        q = 0.5 * air_density * apparent_wind_speed ** 2
        momentum_component = q * self.area
        self.antithrust =  momentum_component * self.ct
        self.power = momentum_component * apparent_wind_speed * self.cp
        self.shaft_speed_hz = (
            self.optimal_lambda * apparent_wind_speed / (math.pi * self.diameter)
        )
        return self

    def inspect(self):
        pitch = self.diameter * self.pitch_to_diameter
        print("=== WindTurbine ===")
        print(f"  Diameter         : {self.diameter.to('inch'):.4g}")
        print(f"  Blade pitch      : {pitch.to('inch'):.4g}")
        print(f"  Pitch/Diameter   : {self.pitch_to_diameter:.4f}")
        print(f"  Swept area       : {self.area.to('inch**2'):.4g}")
        print(f"  Ct               : {self.ct:.4f}")
        print(f"  Cp               : {self.cp:.4f}")
        if self.power is not None:
            print(f"  Apparent wind    : {self.apparent_wind_speed.to('knot'):.4g}")
            print(f"  Shaft speed      : {self.shaft_speed_hz.to('rpm'):.4g}")
            print(f"  Power extracted  : {self.power.to('watt'):.4g}")
            print(f"  Antithrust       : {self.antithrust.to('newton'):.4g}")


class Propeller:
    """
    Simplified propeller model using actuator disk / momentum theory.
    Operates in water; sized from turbine shaft output and boat speed.
    """

    def __init__(
        self,
        effective_power_factor=0.7,
        optimal_slip=0.1,
        water_density=999 * ureg.kg / ureg.meter ** 3,
    ):
        self.effective_power_factor = effective_power_factor
        self.optimal_slip = optimal_slip
        self.water_density = water_density

        # Set at solve time
        self.shaft_speed_hz = None
        self.boat_speed = None
        self.thrust = None
        self.diameter = None
        self.pitch = None
        self.pitch_to_diameter = None

    def solve(self, shaft_power, shaft_speed_hz, boat_speed):
        """Size the propeller and compute thrust from turbine power and boat speed."""
        self.shaft_speed_hz = shaft_speed_hz
        self.boat_speed = boat_speed

        # Slip model (from Wageningen B-series): slip = P/D - J
        # At bollard pull (J=0): P/D = optimal_slip.  At any speed: P/D = optimal_slip + J.
        # Pitch from slip constraint: p = D*(s + J) = D*s + V/n
        # Slip velocity: delta_v = n*p - V = n*D*s  (independent of boat speed — finite at bollard pull)
        #
        # Actuator disk power balance: P_eff = rho * (pi*D^2/4) * V_disk^2 * delta_v
        #   where V_disk = V + delta_v/2 = V + n*D*s/2
        # Solve this cubic-in-D numerically, then back out thrust and geometry.

        P_eff = (shaft_power * self.effective_power_factor).to('watt').magnitude
        n = shaft_speed_hz.to('Hz').magnitude
        V = boat_speed.to('m/s').magnitude
        rho = self.water_density.to('kg/m^3').magnitude
        s = self.optimal_slip

        def power_residual(D):
            V_d = V + n * s * D / 2
            return rho * math.pi * n * s / 4 * D**3 * V_d**2 - P_eff

        D_m = brentq(power_residual, 1e-9, 1e4)

        self.diameter = D_m * ureg.meter
        delta_v = n * s * D_m * ureg.meter / ureg.second
        V_disk = boat_speed + delta_v / 2

        self.thrust = self.water_density * math.pi / 4 * self.diameter**2 * V_disk * delta_v
        self.pitch = s * self.diameter + boat_speed / shaft_speed_hz
        self.pitch_to_diameter = self.pitch / self.diameter

        return self

    def inspect(self):
        print("=== Propeller ===")
        if self.thrust is not None:
            print(f"  Diameter         : {self.diameter.to('inch'):.4g}")
            print(f"  Pitch            : {self.pitch.to('inch'):.4g}")
            print(f"  Pitch/Diameter   : {self.pitch_to_diameter.to(''):.4f}")
            print(f"  Shaft speed      : {self.shaft_speed_hz.to('rpm'):.4g}")
            print(f"  Boat speed       : {self.boat_speed.to('knot'):.4g}")
            print(f"  Thrust           : {self.thrust.to('newton'):.4g}")
        else:
            print("  (not yet solved)")


class BoatHull:
    """
    Drag model for a small, shallow, round-hulled skimmer.
    """

    def __init__(
        self,
        beam=2 * ureg.inch,
        depth=0.5 * ureg.inch,
        area_multiplier=0.33,
        cd=None,
        water_density=999 * ureg.kg / ureg.meter ** 3,
    ):
        self.beam = beam
        self.depth = depth
        self.area_multiplier = area_multiplier
        self.cd = cd
        self.water_density = water_density

        self.frontal_area = area_multiplier * beam * depth

        # Set at solve time
        self.boat_speed = None
        self.drag = None

        # Calibrate Cd
        if self.cd is None:
            self.cd = calibrate_hull_cd(self)

    def solve(self, boat_speed):
        self.boat_speed = boat_speed
        self.drag = (
            0.5 * self.cd * self.frontal_area * self.water_density * boat_speed ** 2
        )
        return self

    def inspect(self):
        print("=== BoatHull ===")
        print(f"  Beam             : {self.beam.to('inch'):.4g}")
        print(f"  Draft            : {self.depth.to('inch'):.4g}")
        print(f"  Frontal area     : {self.frontal_area.to('inch**2'):.4g}")
        print(f"  Cd               : {self.cd:.4f}")
        if self.drag is not None:
            print(f"  Boat speed       : {self.boat_speed.to('knot'):.4g}")
            print(f"  Drag             : {self.drag.to('newton'):.4g}")


# ---------------------------------------------------------------------------
# System-level solver
# ---------------------------------------------------------------------------

class WindDrivenBoat:
    """
    Couples a WindTurbine, Propeller, and BoatHull to find equilibrium speed.
    """

    def __init__(self, turbine: WindTurbine, propeller: Propeller, hull: BoatHull):
        self.turbine = turbine
        self.propeller = propeller
        self.hull = hull

        self.wind_speed = None
        self.boat_speed = None

    def _net_force(self, boat_speed_knots):
        boat_speed = boat_speed_knots * ureg.knot
        apparent_wind = boat_speed + self.wind_speed

        self.turbine.solve(apparent_wind)
        self.hull.solve(boat_speed)
        self.propeller.solve(
            self.turbine.power, self.turbine.shaft_speed_hz, boat_speed
        )

        net = self.propeller.thrust - self.hull.drag - self.turbine.antithrust
        return net

    def solve(self, wind_speed):
        self.wind_speed = wind_speed
        obj = lambda v: self._net_force(v) ** 2
        result = minimize_scalar(obj, bounds=[1e-10, 1e10])
        self.boat_speed = result.x * ureg.knot

        # Final solve at equilibrium to populate all component states
        self._net_force(result.x)
        return self

    def inspect(self):
        print("=== WindDrivenBoat (equilibrium) ===")
        if self.boat_speed is not None:
            print(f"  Wind speed       : {self.wind_speed.to('knot'):.4g}")
            print(f"  Boat speed       : {self.boat_speed.to('knot'):.4g}")
            vmg = self.boat_speed / self.wind_speed
            print(f"  VMG ratio        : {vmg.to('').magnitude:.4f}")
        print()
        self.turbine.inspect()
        print()
        self.propeller.inspect()
        print()
        self.hull.inspect()


# ---------------------------------------------------------------------------
# Hull calibration from trolling-motor data
# ---------------------------------------------------------------------------

def calibrate_hull_cd(
    hull: BoatHull,
    motor_amps=60 * ureg.ampere,
    motor_volts=13 * ureg.volt,
    observed_speed=5 * ureg.mph,
    prop_efficiency=0.85,
) -> float:
    """
    Back-solve hull Cd from a single trolling-motor data point.

    Treat electrical power as shaft power (motor losses unknown).
    At steady state: Drag = P_shaft * eta_prop / V_boat.
    Then: Cd = Drag / (0.5 * rho * A_frontal * V^2).
    """
    # Geometry from the actual boat; Cd calibrated from trolling-motor run:
    #   60 A × 13 V → 5 mph, prop η = 0.85  →  Drag ≈ 297 N  →  Cd ≈ 0.35
    
    motor_power_consumed = motor_amps * motor_volts
    shaft_power_output = motor_power_consumed * 0.9
    drag = shaft_power_output * prop_efficiency / observed_speed
    q = 0.5 * hull.water_density * observed_speed ** 2
    cd = drag / hull.frontal_area / q
    return cd.to("")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    turbine = WindTurbine(diameter= 630*2 * ureg.mm, axial_induction=0.3333)
    propeller = Propeller(effective_power_factor=0.85)

    hull = BoatHull(beam=48*ureg.inch, depth=12*ureg.inch, area_multiplier=0.8)

    boat = WindDrivenBoat(turbine, propeller, hull)
    boat.solve(wind_speed=5 * ureg.knot)
    boat.inspect()


if __name__ == "__main__":
    main()