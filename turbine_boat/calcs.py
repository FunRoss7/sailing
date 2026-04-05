#!/usr/bin/env python3
import pint
import math

# Create a unit registry
ureg = pint.UnitRegistry()

air_density = 1.225 * ureg.kilogram / ureg.meter**3
water_density = 999 * ureg.kilogram / ureg.meter**3

# turbine assumptions (operating at optimal design point, but the turbine sucks b/c it's a propeller operating in reverse)
optimal_lambda = 2*math.pi # idk, maybe 8? i'm starting with a 2 blade turbine
turbine_pitch_to_diameter = math.pi / optimal_lambda

axial_induction = 0.2 # idk, i just made up a number. there's a better way to do this using Cp max and stuff

# propeller assumptions
propeller_effective_power_factor = 0.7 # conservative estimate i hope
propeller_optimal_slip = 0.1


def wind_turbine(turbine_diameter, apparent_wind_speed):
    optimal_lambda = 2 * math.pi
    axial_induction = 0.2

    area = math.pi * (turbine_diameter / 2)**2

    ct = 4 * axial_induction * (1 - axial_induction)
    cp = 4 * axial_induction * (1 - axial_induction)**2

    antithrust = 0.5 * air_density * area * apparent_wind_speed**2 * ct
    power = 0.5 * air_density * area * apparent_wind_speed**3 * cp
    shaft_speed_hz = optimal_lambda * apparent_wind_speed / (math.pi * turbine_diameter)

    return antithrust, power, shaft_speed_hz

def calculate_boat_drag(boat_speed):
  # geometry properties
  beam = 2 * ureg.inch
  depth = 0.5 * ureg.inch
  area_multiplier = 0.33 # b/c it's a round shallow boat
  Cd = 0.1 # it's also a skimmer

  # calculate drag
  frontal_area = area_multiplier * beam * depth
  drag_force = 1/2*Cd*frontal_area*water_density*boat_speed**2
  return drag_force

def propeller_thrust(power,boat_speed):
  effective_power = propeller_effective_power_factor * power
  thrust = effective_power / boat_speed
  return thrust

def calculate_thrusts(turbine_diameter, wind_speed, boat_speed, select_prop=False):
  apparent_wind_speed = boat_speed + wind_speed
  antithrust, power, shaft_speed_hz = wind_turbine(turbine_diameter, apparent_wind_speed)
  drag = calculate_boat_drag(boat_speed)
  thrust = propeller_thrust(power, boat_speed)

  # calculate matching propeller properties
  if select_prop:
    # pitch is simple
    prop_pitch = boat_speed/shaft_speed_hz * (propeller_optimal_slip + 1)

    # diameter takes a bit more calculation
    delta_v = propeller_optimal_slip * boat_speed
    m_dot = thrust / delta_v
    area = m_dot / boat_speed / water_density
    radius = (area / math.pi)**0.5
    prop_diameter = radius*2

    return {'pitch': prop_pitch/ prop_diameter, 'diameter':prop_diameter.to('mm'), "shaft speed":shaft_speed_hz.to('rpm')}


  return thrust - drag - antithrust

def calculate_vessel_speed(turbine_diameter, wind_speed):
  minimizer = lambda b: calculate_thrusts(turbine_diameter, wind_speed, b * ureg.knot) **  2
  boat_speed = minimize_scalar(minimizer, bounds=[1e-10,1e10]).x
  boat_speed = boat_speed * ureg.knot
  propeller = calculate_thrusts(turbine_diameter, wind_speed, boat_speed, select_prop=True)
  return boat_speed, propeller

def main():
  #w = np.linspace(1*ureg.knot,10*ureg.knot,10)
  turbine_diameter = 8 * ureg.inch
  turbine_pitch = turbine_diameter*turbine_pitch_to_diameter
  w = 10 * ureg.knot
  b = calculate_vessel_speed(turbine_diameter,w)
  print("turbine diameter:",turbine_diameter)
  print("turbine pitch:",turbine_pitch)
  print(w)
  print(b)


main()




