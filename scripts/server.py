#!/usr/bin/env python

from cse571_project.srv import *
import rospy
from mazeGenerator import *
import sys
import argparse
import time
from action_server import RobotActionsServer 
import pickle
import copy
import os

root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
books = None
mazeInfo = None
mazeInfoCopy = None
parser = argparse.ArgumentParser()
parser.add_argument('-t', help='for specifying number of trucks', metavar='5', action='store', dest='n_trucks', default=4, type=int)
parser.add_argument('-p', help='for specifying number of packages', metavar='5', action='store', dest='n_packages', default=20, type=int)
parser.add_argument('-d', help='for specifying number of depots', metavar='5', action='store', dest='n_depots', default=3, type=int)
parser.add_argument('-g', help='for specifying grid scale', metavar='5', action='store', dest='grid_scale', default=1.0, type=float)
parser.add_argument('-s', help='for providing random seed', metavar='32', action='store', dest='seed', default=int(time.time()), type=int)
robot_action_server = None


def check_is_edge(req):
	"""
	This function checks if two points are connected via edge or not.
	"""
	global mazeInfo
	edge = (req.x1,req.y1,req.x2,req.y2)
	for edge_point in edge:
		if edge_point < mazeInfo.grid_start or edge_point > mazeInfo.grid_dimension * 0.5:
			return 0
	if edge in mazeInfo.blocked_edges or (edge[2],edge[3],edge[0],edge[1]) in mazeInfo.blocked_edges:
		return 0
	else:
		return 1


def handle_reset_world(req):
	global mazeInfo
	mazeInfo = copy.deepcopy(mazeInfoCopy)
	robot_action_server.current_state = robot_action_server.generate_init_state()
	return 1


def remove_blocked_edge(req):
	bookname = req.bookname
	global books
	global mazeInfo
	location_of_blocked_edge_list = books["books"][bookname]["load_loc"]
	if location_of_blocked_edge_list[0][0] <= location_of_blocked_edge_list[1][0] and location_of_blocked_edge_list[0][1] <= location_of_blocked_edge_list[1][1]:
		blocked_edge = (location_of_blocked_edge_list[0][0], location_of_blocked_edge_list[0][1], location_of_blocked_edge_list[1][0], location_of_blocked_edge_list[1][1])
	else:
		blocked_edge = (location_of_blocked_edge_list[1][0], location_of_blocked_edge_list[1][1], location_of_blocked_edge_list[0][0], location_of_blocked_edge_list[0][1])
	mazeInfo.blocked_edges.remove(blocked_edge)
	return "1"


def server():
	rospy.Service('remove_blocked_edge', RemoveBlockedEdgeMsg,remove_blocked_edge)
	rospy.Service('check_is_edge',CheckEdge,check_is_edge)
	rospy.Service('reset_world',ResetWorldMsg,handle_reset_world)
	print "Ready!"
	rospy.spin()


if __name__ == "__main__":
	args = parser.parse_args()
	n_trucks = args.n_trucks
	n_packages = args.n_packages
	n_depots = args.n_depots
	seed = args.seed
	print("Number of Trucks: %d, Number of Packages: %d, Number of Depots: %d, Seed: %d" %(n_trucks, n_packages, n_depots, seed))
	if n_packages < n_depots or n_trucks < n_depots:
		print("Please enter more trucks/ depots")
		exit()
	grid_size = 3 * n_depots * args.grid_scale
	mazeInfo = Maze(grid_size, 0.5)
	books = mazeInfo.generate(n_trucks, n_depots, n_packages, seed, root_path)
	mazeInfoCopy = copy.deepcopy(mazeInfo)
	print "blocked_edges: ", mazeInfo.blocked_edges
	rospy.init_node('server')
	# robot_action_server = RobotActionsServer(books, root_path, args.action_seed)
	server()
