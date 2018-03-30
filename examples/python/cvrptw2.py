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

from __future__ import print_function
import sys
from six.moves import xrange
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

# Distance callback

class CreateDistanceCallback(object): # pylint: disable=too-few-public-methods
    """Creates callback to calculate distances and travel times between points."""

    @staticmethod
    def manhattan_distance(x_1, y_1, x_2, y_2):
        """Computes the Manhattan distance between [x_1, y_1] and [x_2, y_2]"""
        return abs(x_1 - x_2) + abs(y_1 - y_2)

    def __init__(self, locations):
        """Initializes the distance matrix."""
        self.matrix = {}

        num_locations = len(locations)
        for from_node in xrange(num_locations):
            self.matrix[from_node] = {}
            for to_node in xrange(num_locations):
                if from_node == to_node:
                    self.matrix[from_node][to_node] = 0
                else:
                    x_1 = locations[from_node][0]
                    y_1 = locations[from_node][1]
                    x_2 = locations[to_node][0]
                    y_2 = locations[to_node][1]
                    self.matrix[from_node][to_node] = self.manhattan_distance(x_1, y_1, x_2, y_2)

    def distance(self, from_node, to_node):
        """Returns the manhattan distance between the two nodes"""
        return self.matrix[from_node][to_node]


# Demand callback
class CreateDemandCallback(object): # pylint: disable=too-few-public-methods
    """Creates callback to get demands at each location."""

    def __init__(self, demands):
        """Initializes the demand array."""
        self.demands = demands

    def demand(self, from_node, to_node):
        """Returns the demand of the current node"""
        del to_node
        return self.demands[from_node]

# Service time (proportional to demand) callback.
class CreateServiceTimeCallback(object): # pylint: disable=too-few-public-methods
    """Creates callback to get service time at each location."""

    def __init__(self, demands, time_per_demand_unit):
        """Initializes the time service array."""
        self.service_times = [demand * time_per_demand_unit for demand in demands]

    def service_time(self, from_node, to_node):
        """Returns the service time of the current node"""
        del to_node
        return self.service_times[from_node]

# Create the travel time callback (equals distance divided by speed).
class CreateTravelTimeCallback(object): # pylint: disable=too-few-public-methods
    """Creates callback to get travel times between locations."""

    def __init__(self, locations, distance, speed):
        """Initializes the travel time matrix."""
        self.matrix = {}

        num_locations = len(locations)
        for from_node in xrange(num_locations):
            self.matrix[from_node] = {}
            for to_node in xrange(num_locations):
                if from_node == to_node:
                    self.matrix[from_node][to_node] = 0
                else:
                    self.matrix[from_node][to_node] = int(distance(from_node, to_node) / speed)

    def travel_time(self, from_node, to_node):
        """Returns the travel time between the two nodes"""
        return self.matrix[from_node][to_node]

# Create total_time callback (equals service time plus travel time).
class CreateTotalTimeCallback(object): # pylint: disable=too-few-public-methods
    """Creates callback to get total times between locations."""

    def __init__(self, locations, service_time_callback, travel_time_callback):
        """Initializes the total time matrix."""
        self.matrix = {}

        num_locations = len(locations)
        for from_node in xrange(num_locations):
            self.matrix[from_node] = {}
            for to_node in xrange(num_locations):
                if from_node == to_node:
                    self.matrix[from_node][to_node] = 0
                else:
                    self.matrix[from_node][to_node] = \
                      service_time_callback(from_node, to_node) + \
                      travel_time_callback(from_node, to_node)

    def total_time(self, from_node, to_node):
        """Returns the total time between the two nodes"""
        return self.matrix[from_node][to_node]

def main():
    """Entry point of the program"""
    # Create the data.
    [locations, demands, start_times, end_times] = create_data_array()
    num_locations = len(locations)

    if num_locations == 0:
        print('Specify an instance greater than 0.')
        sys.exit(1)

    # Create routing model.
    # The number of nodes of the VRP is num_locations.
    # Nodes are indexed from 0 to num_locations - 1.
    # By default the start of a route is node 0.
    num_vehicles = 5
    depot = 0
    routing = pywrapcp.RoutingModel(num_locations, num_vehicles, depot)
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()
    # Setting first solution heuristic (cheapest addition).
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Callbacks to the distance function.
    dist_between_locations = CreateDistanceCallback(locations)
    dist_callback = dist_between_locations.distance
    routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)

    # Adding capacity dimension constraints.
    demand_at_locations = CreateDemandCallback(demands)
    demands_callback = demand_at_locations.demand
    vehicle_capacity = 100
    null_capacity_slack = 0
    fix_start_cumul_to_zero = True
    capacity = "Capacity"
    routing.AddDimension(demands_callback,
                         null_capacity_slack,
                         vehicle_capacity,
                         fix_start_cumul_to_zero,
                         capacity)

    # Adding a dimension for time-window constraints and limits on the start times and end times.
    # Time to deliver a package to a customer: 3min/unit
    time_per_demand_unit = 3 * 60
    service_times = CreateServiceTimeCallback(demands, time_per_demand_unit)
    service_time_callback = service_times.service_time

    # Travel speed: 80km/h to convert in km/s
    speed = 80 / 3600.
    travel_times = CreateTravelTimeCallback(locations, dist_callback, speed)
    travel_time_callback = travel_times.travel_time

    total_times = CreateTotalTimeCallback(locations, service_time_callback, travel_time_callback)
    total_time_callback = total_times.total_time

    horizon = 24 * 3600
    time = "Time"
    routing.AddDimension(total_time_callback,
                         horizon,
                         horizon,
                         fix_start_cumul_to_zero,
                         time)

    time_dimension = routing.GetDimensionOrDie(time)
    for order in xrange(1, num_locations):
        start = start_times[order]
        end = end_times[order]
        time_dimension.CumulVar(order).SetRange(start, end)

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    # Display a solution if any.
    if assignment:
        # Solution cost.
        print("Total distance of all routes: ", str(assignment.ObjectiveValue()), '\n')
        # Inspect solution.
        capacity_dimension = routing.GetDimensionOrDie(capacity)
        time_dimension = routing.GetDimensionOrDie(time)

        for vehicle_nbr in xrange(num_vehicles):
            index = routing.Start(vehicle_nbr)
            plan_output = 'Route for vehicle {0}:\n'.format(vehicle_nbr)
            route_dist = 0

            while not routing.IsEnd(index):
                node_index = routing.IndexToNode(index)
                route_dist += dist_callback(node_index,
                                            routing.IndexToNode(
                                                assignment.Value(routing.NextVar(index))))
                load_var = capacity_dimension.CumulVar(index)
                time_var = time_dimension.CumulVar(index)
                plan_output += " {node_index} Load({load}) Time({tmin}, {tmax}) -> ".format(
                    node_index=node_index,
                    load=assignment.Value(load_var),
                    tmin=str(assignment.Min(time_var)),
                    tmax=str(assignment.Max(time_var)))
                index = assignment.Value(routing.NextVar(index))

            node_index = routing.IndexToNode(index)
            load_var = capacity_dimension.CumulVar(index)
            time_var = time_dimension.CumulVar(index)
            plan_output += " {node_index} Load({load}) Time({tmin}, {tmax})\n".format(
                node_index=node_index,
                load=assignment.Value(load_var),
                tmin=str(assignment.Min(time_var)),
                tmax=str(assignment.Max(time_var)))
            plan_output += 'Distance of the route {0}: {dist}\n'.format(
                vehicle_nbr,
                dist=route_dist)
            plan_output += 'Demand met by vehicle {0}: {load}\n'.format(
                vehicle_nbr,
                load=assignment.Value(load_var))
            print(plan_output, '\n')
    else:
        print('No solution found.')
        sys.exit(2)

def create_data_array():
    """Creates the data for the problem"""
    locations = [[82, 76], [96, 44], [50, 5], [49, 8], [13, 7], [29, 89], [58, 30], [84, 39],
                 [14, 24], [12, 39], [3, 82], [5, 10], [98, 52], [84, 25], [61, 59], [1, 65],
                 [88, 51], [91, 2], [19, 32], [93, 3], [50, 93], [98, 14], [5, 42], [42, 9],
                 [61, 62], [9, 97], [80, 55], [57, 69], [23, 15], [20, 70], [85, 60], [98, 5]]

    demands = [0, 19, 21, 6, 19, 7, 12, 16,
               6, 16, 8, 14, 21, 16, 3, 22,
               18, 19, 1, 24, 8, 12, 4, 8,
               24, 24, 2, 20, 15, 2, 14, 9]

    start_times = [0, 5080, 1030, 4930, 2250, 5310, 890, 5650,
                   5400, 1080, 6020, 4660, 3560, 3030, 3990, 3820,
                   3620, 5210, 230, 4890, 4450, 3180, 3800, 550,
                   5740, 5150, 1100, 3100, 3870, 4910, 3280, 730]

    # The width of the time window: 5 hours.
    tw_duration = 5 * 60 * 60

    # In this example, the time window widths is the same at each location, so we define the end
    # times to be start times + tw_duration.
    # For problems in which the time window widths vary by location, you can explicitly define the
    # list of end_times, as we have done for start_times.
    end_times = [start + tw_duration for start in start_times]

    return [locations, demands, start_times, end_times]

if __name__ == '__main__':
    main()
