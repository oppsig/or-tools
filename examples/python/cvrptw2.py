# This Python file uses the following encoding: utf-8
# Copyright 2015 Tin Arm Engineering AB
# Copyright 2017 Google LLC
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Capacitated Vehicle Routing Problem with Time Windows.

   This is a sample using the routing library python wrapper to solve a
   CVRPTW problem.
   A description of the problem can be found here:
   http://en.wikipedia.org/wiki/Vehicle_routing_problem.
   The variant which is tackled by this model includes a capacity dimension
   and time windows.
   Distances are computed using the Manhattan distances. Distances are in km
   and times in seconds.


   The optimization engine uses local search to improve solutions, first
   solutions being generated using a cheapest addition heuristic.
"""
from six.moves import xrange
from ortools.constraint_solver import pywrapcp

def distance(x1, y1, x2, y2):
    # Manhattan distance
    dist = abs(x1 - x2) + abs(y1 - y2)
    return dist

# Distance callback

class CreateDistanceCallback(object):
  """Create callback to calculate distances and travel times between points."""

  def __init__(self, locations):
    """Initialize distance array."""
    num_locations = len(locations)
    self.matrix = {}

    for from_node in xrange(num_locations):
      self.matrix[from_node] = {}
      for to_node in xrange(num_locations):
        if from_node == to_node:
          self.matrix[from_node][to_node] = 0
        else:
          x1 = locations[from_node][0]
          y1 = locations[from_node][1]
          x2 = locations[to_node][0]
          y2 = locations[to_node][1]
          self.matrix[from_node][to_node] = distance(x1, y1, x2, y2)

  def Distance(self, from_node, to_node):
    return self.matrix[from_node][to_node]


# Demand callback
class CreateDemandCallback(object):
  """Create callback to get demands at each location."""

  def __init__(self, demands):
    self.matrix = demands

  def Demand(self, from_node, to_node):
    return self.matrix[from_node]

# Service time (proportional to demand) callback.
class CreateServiceTimeCallback(object):
  """Create callback to get time windows at each location."""

  def __init__(self, demands, time_per_demand_unit):
    self.matrix = demands
    self.time_per_demand_unit = time_per_demand_unit

  def ServiceTime(self, from_node, to_node):
    return self.matrix[from_node] * self.time_per_demand_unit

# Create the travel time callback (equals distance divided by speed).
class CreateTravelTimeCallback(object):
  """Create callback to get travel times between locations."""

  def __init__(self, dist_callback, speed):
    self.dist_callback = dist_callback
    self.speed = speed

  def TravelTime(self, from_node, to_node):
    travel_time = self.dist_callback(from_node, to_node) / self.speed
    return int(travel_time)

# Create total_time callback (equals service time plus travel time).
class CreateTotalTimeCallback(object):
  """Create callback to get total times between locations."""

  def __init__(self, service_time_callback, travel_time_callback):
    self.service_time_callback = service_time_callback
    self.travel_time_callback = travel_time_callback

  def TotalTime(self, from_node, to_node):
    service_time = self.service_time_callback(from_node, to_node)
    travel_time = self.travel_time_callback(from_node, to_node)
    return service_time + travel_time

def main():
  # Create the data.
  data = create_data_array()
  locations = data[0]
  demands = data[1]
  start_times = data[2]
  end_times = data[3]
  num_locations = len(locations)
  depot = 0
  num_vehicles = 5
  search_time_limit = 400000

  # Create routing model.
  if num_locations > 0:

    # The number of nodes of the VRP is num_locations.
    # Nodes are indexed from 0 to num_locations - 1. By default the start of
    # a route is node 0.
    routing = pywrapcp.RoutingModel(num_locations, num_vehicles, depot)
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()

    # Callbacks to the distance function and travel time functions here.
    dist_between_locations = CreateDistanceCallback(locations)
    dist_callback = dist_between_locations.Distance
    routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)

    # Adding capacity dimension constraints.
    demands_at_locations = CreateDemandCallback(demands)
    demands_callback = demands_at_locations.Demand
    VehicleCapacity = 100;
    NullCapacitySlack = 0;
    fix_start_cumul_to_zero = True
    capacity = "Capacity"
    routing.AddDimension(demands_callback, NullCapacitySlack, VehicleCapacity,
                         fix_start_cumul_to_zero, capacity)

    # Add time dimension constraints.
    # Time to deliver a package to a customer: ~3min/unit
    time_per_demand_unit = 3 * 60
    horizon = 24 * 3600
    time = "Time"
    # Travel speed: 80Km/h
    speed = 80 / 3600

    service_times = CreateServiceTimeCallback(demands, time_per_demand_unit)
    service_time_callback = service_times.ServiceTime

    travel_times = CreateTravelTimeCallback(dist_callback, speed)
    travel_time_callback = travel_times.TravelTime

    total_times = CreateTotalTimeCallback(service_time_callback, travel_time_callback)
    total_time_callback = total_times.TotalTime

    # Add a dimension for time-window constraints and limits on the start times and end times.
    routing.AddDimension(total_time_callback,  # total time function callback
                         horizon,
                         horizon,
                         fix_start_cumul_to_zero,
                         time)

    # Add time window constraints.
    time_dimension = routing.GetDimensionOrDie(time)

    for order in xrange(1, num_locations):
      start = start_times[order]
      end = end_times[order]
      time_dimension.CumulVar(order).SetRange(start, end)

    # Solve displays a solution if any.
    assignment = routing.SolveWithParameters(search_parameters)
    if assignment:
      size = len(locations)
      # Solution cost.
      print ("Total distance of all routes: " , str(assignment.ObjectiveValue()), '\n')
      # Inspect solution.
      capacity_dimension = routing.GetDimensionOrDie(capacity);
      time_dimension = routing.GetDimensionOrDie(time);

      for vehicle_nbr in xrange(num_vehicles):
        index = routing.Start(vehicle_nbr)
        plan_output = 'Route for vehicle {0}:\n'.format(vehicle_nbr)
        route_dist = 0

        while not routing.IsEnd(index):
          node_index = routing.IndexToNode(index)
          route_dist += dist_callback(node_index, routing.IndexToNode(assignment.Value(routing.NextVar(index))))
          load_var = capacity_dimension.CumulVar(index)
          time_var = time_dimension.CumulVar(index)
          plan_output += \
                    " {node_index} Load({load}) Time({tmin}, {tmax}) -> ".format(
                        node_index=node_index,
                        load=assignment.Value(load_var),
                        tmin=str(assignment.Min(time_var)),
                        tmax=str(assignment.Max(time_var)))
          index = assignment.Value(routing.NextVar(index))

        node_index = routing.IndexToNode(index)
        load_var = capacity_dimension.CumulVar(index)
        time_var = time_dimension.CumulVar(index)
        plan_output += \
                  " {node_index} Load({load}) Time({tmin}, {tmax})\n".format(
                      node_index=node_index,
                      load=assignment.Value(load_var),
                      tmin=str(assignment.Min(time_var)),
                      tmax=str(assignment.Max(time_var)))
        plan_output += 'Distance of the route {0}: {dist}\n'.format(vehicle_nbr, dist=route_dist)
        plan_output += 'Demand met by vehicle {0}: {load}\n'.format(vehicle_nbr, load=assignment.Value(load_var))
        print (plan_output , '\n')
    else:
      print ('No solution found.')
  else:
    print ('Specify an instance greater than 0.')

def create_data_array():

  locations = [[82, 76], [96, 44], [50,  5], [49,  8], [13,  7], [29, 89], [58, 30], [84, 39],
               [14, 24], [12, 39], [ 3, 82], [ 5, 10], [98, 52], [84, 25], [61, 59], [ 1, 65],
               [88, 51], [91,  2], [19, 32], [93,  3], [50, 93], [98, 14], [ 5, 42], [42,  9],
               [61, 62], [ 9, 97], [80, 55], [57, 69], [23, 15], [20, 70], [85, 60], [98,  5]]

  demands =   [       0,       19,       21,        6,       19,        7,       12,       16,
                      6,       16,        8,       14,       21,       16,        3,       22,
                     18,       19,        1,       24,        8,       12,        4,        8,
                     24,       24,        2,       20,       15,        2,       14,        9]

  start_times = [     0,     5080,     1030,     4930,     2250,     5310,      890,     5650,
                   5400,     1080,     6020,     4660,     3560,     3030,     3990,     3820,
                   3620,     5210,      230,     4890,     4450,     3180,     3800,      550,
                   5740,     5150,     1100,     3100,     3870,     4910,     3280,      730]

  # tw_duration is the width of the time windows (here 5 hours).
  tw_duration = 5 * 60 * 60

  # In this example, the width is the same at each location, so we define the end times to be
  # start times + tw_duration. For problems in which the time window widths vary by location,
  # you can explicitly define the list of end_times, as we have done for start_times.
  end_times = [0] * len(start_times)

  for i in range(len(start_times)):
    end_times[i] = start_times[i] + tw_duration

  data = [locations, demands, start_times, end_times]
  return data

if __name__ == '__main__':
  main()
