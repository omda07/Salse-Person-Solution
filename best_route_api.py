from flask import Flask
from flask_restful import Resource, Api, reqparse
import pandas as pd
import ast


from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

#####################################
import requests
import json
import urllib.request


def create_data():
    """Creates the data."""
    data = {}
    data['API_key'] = 'AIzaSyAAzZ23wiCv3nBGsLNmzJfXIV2O3GBT-fI'
    data['addresses'] = ['29.982900,31.321331',
                         '30.032718,31.410320',
                         '29.961815,31.302602',
                         '30.070595,31.45298']
    return data


def create_distance_matrix(data):
    addresses = data["addresses"]
    API_key = data["API_key"]
    # Distance Matrix API only accepts 100 elements per request, so get rows in multiple requests.
    max_elements = 100
    num_addresses = len(addresses)  # 16 in this example.
    # Maximum number of rows that can be computed per request (6 in this example).
    max_rows = max_elements // num_addresses
    # num_addresses = q * max_rows + r (q = 2 and r = 4 in this example).
    q, r = divmod(num_addresses, max_rows)
    dest_addresses = addresses
    distance_matrix = []
    # Send q requests, returning max_rows rows per request.
    for i in range(q):
        origin_addresses = addresses[i * max_rows: (i + 1) * max_rows]
        response = send_request(origin_addresses, dest_addresses, API_key)
        distance_matrix += build_distance_matrix(response)

    # Get the remaining remaining r rows, if necessary.
    if r > 0:
        origin_addresses = addresses[q * max_rows: q * max_rows + r]
        response = send_request(origin_addresses, dest_addresses, API_key)
        distance_matrix += build_distance_matrix(response)
    return distance_matrix


def send_request(origin_addresses, dest_addresses, API_key):
    """ Build and send request for the given origin and destination addresses."""

    def build_address_str(addresses):
        # Build a pipe-separated string of addresses
        address_str = ''
        for i in range(len(addresses) - 1):
            address_str += addresses[i] + '|'
        address_str += addresses[-1]
        return address_str

    request = 'https://maps.googleapis.com/maps/api/distancematrix/json?'
    origin_address_str = build_address_str(origin_addresses)
    dest_address_str = build_address_str(dest_addresses)
    request = request + '&origins=' + origin_address_str + '&destinations=' + \
        dest_address_str + '&key=' + API_key
    jsonResult = urllib.request.urlopen(request).read()
    response = json.loads(jsonResult)
    return response


def build_distance_matrix(response):
    distance_matrix = []
    for row in response['rows']:
        row_list = [row['elements'][j]['distance']['value']
                    for j in range(len(row['elements']))]
        distance_matrix.append(row_list)
    return distance_matrix


def create_data_model():
    """Stores the data for the problem."""
    data = create_data()
    addresses = data['addresses']
    API_key = data['API_key']
    distance_matrix = create_distance_matrix(data)
    print('disrabe')
    print(distance_matrix)
    print('disrabe')

    data0 = {}
    data0['distance_matrix'] = distance_matrix
    print(data0)
    data0['num_vehicles'] = 1
    data0['depot'] = 0
    return data0


def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    address = ['29.982900,31.321331',
               '30.032718,31.410320',
               '29.961815,31.302602',
               '30.070595,31.45298']
    sortedAddress = []
    max_route_distance = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output1 = '{}:'.format(vehicle_id)
        route_distance = 0
        plan_output = ''
        while not routing.IsEnd(index):

            plan_output += '{} -> '.format(manager.IndexToNode(index))
            sortedAddress.append(address[index])

            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += '{}'.format(manager.IndexToNode(index))
        sortedAddress.append(address[0])
        plan_output2 = '{}k'.format(route_distance/1000)
        print(plan_output)
        print(sortedAddress)
        max_route_distance = max(route_distance, max_route_distance)
    print('Maximum of the route distances: {}k'.format(max_route_distance/1000))

    return {
        "address": sortedAddress,
        "route_for_vehicle": plan_output1,
        "path": plan_output,
        "distance_of_the_route": plan_output2
    }


app = Flask(__name__)
api = Api(app)


class Location(Resource):

    def get(self):

        data1 = create_data_model()

        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(len(data1['distance_matrix']),
                                               data1['num_vehicles'], data1['depot'])

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        # Create and register a transit callback.

        def distance_callback(from_index, to_index):
            """Returns the distance between the two nodes."""
            # Convert from routing variable Index to distance matrix NodeIndex.
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return data1['distance_matrix'][from_node][to_node]

        transit_callback_index = routing.RegisterTransitCallback(
            distance_callback)
        # Define cost of each arc.
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Add Distance constraint.
        dimension_name = 'Distance'
        routing.AddDimension(
            transit_callback_index,
            0,  # no slack
            300000000,  # vehicle maximum travel distance
            True,  # start cumul to zero
            dimension_name)
        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(100)

        # Setting first solution heuristic.
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

        # Solve the problem.
        solution = routing.SolveWithParameters(search_parameters)

        # Print solution on console.
        if solution:
            return {'status': 200, 'data': print_solution(data1, manager, routing, solution)}, 200
        else:
            print('No solution found !')
        return {'data': 'No solution found !'}, 200


api.add_resource(Location, '/location')


if __name__ == '__main__':
    app.run()
