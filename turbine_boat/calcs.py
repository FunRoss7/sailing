#!/usr/bin/env python3
import pint
import math

# Create a unit registry
ureg = pint.UnitRegistry()

air_density = 1.225 * ureg.kilogram / ureg.meter**3

# frame dynamics
wind_speed = 10 * ureg.knots
boat_speed = 5 * ureg.knots

# frame calcs
apparent_wind_speed = wind_speed + boat_speed

# geometry
turbine_diameter = 11 * ureg.inch
turbine_radius = turbine_diameter / 2

# geometry calcs
swept_area = math.pi * turbine_radius**2
max_cp_design_pt, lambda_design_pt = find_max_cp()

# anti thrust calcs here

# convert to contextual units
shaft_speed_omega = (lambda_design_pt * apparent_wind_speed) / turbine_radius
mechanical_power = 0.5 * air_density * swept_area * apparent_wind_speed**3 * max_cp_design_pt

# Convert to a more readable units
shaft_speed_rpm = shaft_speed_omega.to(ureg.rpm)
mechanical_power_watts = mechanical_power.to(ureg.watt)

# Convert to a common power unit like watts or kilowatts
print(f"Shaft Speed (Rotor Speed): {shaft_speed_rpm:.2f}")
print(f"Mechanical Power: {mechanical_power_watts:.2f}")

# Calculate torque: P = T * omega => T = P / omega
torque = mechanical_power / shaft_speed_omega
torque_nm = torque.to(ureg.newton * ureg.meter)
print(f"Torque: {torque_nm:.2f}")

# Calculate thrust
# Step 1: Solve for axial induction factor 'a' from Cp = 4a(1-a)^2
# Equation: 4a^3 - 8a^2 + 4a - Cp_design_pt = 0
coefficients = [4, -8, 4, -max_cp_design_pt]
roots = np.roots(coefficients)

# Find the real root 'a' between 0 and 0.5 (typical range for axial induction factor)
axial_induction_factor_a = None
for root in roots:
    if np.isreal(root) and (root.real > 0) and (root.real < 0.5):
        axial_induction_factor_a = root.real
        break

if axial_induction_factor_a is None:
    print("Could not find a valid axial induction factor 'a' for the given Cp.")
    thrust = None
else:
    # Step 2: Calculate the thrust coefficient (Ct) using Ct = 4a(1-a)
    thrust_coefficient_Ct = 4 * axial_induction_factor_a * (1 - axial_induction_factor_a)

    # Step 3: Calculate the thrust: T = 0.5 * air_density * swept_area   * apparent_wind_speed**2 * thrust_coefficient_Ct
    thrust = 0.5 * air_density * swept_area * apparent_wind_speed**2 * thrust_coefficient_Ct
    thrust_newtons = thrust.to(ureg.newton)
    print(f"Thrust: {thrust_newtons:.2f}")

