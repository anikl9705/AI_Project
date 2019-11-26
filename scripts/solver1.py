#!/usr/bin/env python

from group_4.srv import *
import rospy
import yaml
import json
import os
import random
import numpy as np
import heapq
import argparse
import Queue as queue
import itertools
import math
import pickle
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose

ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
SAVE_FILE = ROOT_PATH + "/" + "plan_path.pkl"
LOAD_FILE = ROOT_PATH + "/" + "temp_env.json"
parser = argparse.ArgumentParser()
parser.add_argument('-f', help='for specifying environment file', metavar='5', action='store', dest='LOAD_FILE', default='temp_env.json', type=str)

def distanceCalc(x1, x2, y1, y2, typ):
    if typ == "Manhattan":
        return abs(x1-x2) + abs(y1-y2)

class Solver:
    def __init__(self):
        # self.environment = environment
        with open(LOAD_FILE) as json_file:
            try:
                env_json = json.load(json_file, parse_float=float)
                self.depots = env_json["Depots"]
            except (ValueError, KeyError, TypeError):
                print("JSON error")
    
    def cluster(self, depot_no, mode="kmeans"):
        packages = self.depots[depot_no]["Packages"]
        trucks = self.depots[depot_no]["Trucks"]
        if len(trucks) == 0:
            return []
        trucks = sorted(trucks, key=lambda x: x["Capacity"], reverse=True)

        if mode == "kmeans":
            clusters = self.k_means(trucks, packages)
        self.depots[depot_no]["Packages"] = clusters[-1]
        self.depots[depot_no]["Trucks"] = trucks
        return clusters[:-1]
    
    def k_means(self, trucks, packages):
        n = len(packages)
        k = min(len(trucks), n)
        capacities = np.zeros((k))
        initial_members = random.sample(range(n), k)
        centers = np.zeros((k, 2))
        i = 0
        for member in initial_members:
            centers[i][0] = packages[member]["x"]
            centers[i][1] = packages[member]["y"]
            i += 1
        distances = np.zeros((n, k))
        distance_prefs = np.zeros((n, k), dtype=int)
        cluster_alloc = np.zeros((n), dtype=int)
        for i in range(k):
            capacities[i] = trucks[i]["Capacity"]

        isUpdate = True
        while isUpdate:
            isUpdate = False
            for i in range(n):
                for j in range(k):
                    distance = distanceCalc(packages[i]["x"], centers[j][0], packages[i]["y"], centers[j][1], "Manhattan")
                    distances[i][j] = distance
                preferences = np.argsort(distances[i])
                distance_prefs[i] = preferences
                # print(preferences)

            current_capacities = np.zeros((k))
            clusters = [[] for i in range(k)]
            eval_list = queue.Queue()
            for i in range(n):
                eval_list.put((i, 0))
            while not eval_list.empty():
                item, no = eval_list.get()
                while no < k:
                    preference = distance_prefs[item][no]
                    if len(clusters[preference]) != 0:
                        if current_capacities[preference] + packages[item]["Size"] <= capacities[preference]:
                            heapq.heappush(clusters[preference], (distances[item][preference], item, no))
                            current_capacities[preference] += packages[item]["Size"]
                        else:
                            min_dist, min_item, min_no = clusters[preference][0]
                            if min_dist > distances[item][preference]:
                                if current_capacities[preference] - packages[min_item]["Size"] + packages[item]["Size"] <= capacities[preference]:
                                    heapq.heappop(clusters[preference])
                                    heapq.heappush(clusters[preference], (distances[item][preference], item, no))
                                    if min_no < k-1:
                                        eval_list.put((min_item, min_no+1))
                    else:
                        if current_capacities[preference] + packages[item]["Size"] <= capacities[preference]:
                            heapq.heappush(clusters[preference], (distances[item][preference], item, no))
                            current_capacities[preference] += packages[item]["Size"]
                    no += 1

                                
            new_centers = np.zeros((k, 2))
            for i in range(k):
                for j in range(len(clusters[i])):
                    dist, item, no = clusters[i][j]
                    new_centers[i][0] += packages[item]["x"]
                    new_centers[i][1] += packages[item]["y"]
                new_centers[i] /= len(clusters[i])
                new_centers[i] /= len(clusters[i])

            for i in range(k):
                if new_centers[i][0] != centers[i][0] or new_centers[i][1] != centers[i][1]:
                    isUpdate = True
            centers = new_centers

        cluster_results = [[] for i in range(k+1)]
        is_selected = np.full((n), False, dtype=bool)

        for i in range(k):
            for j in range(len(clusters[i])):
                dist, item, no = clusters[i][j]
                is_selected[item] = True
                cluster_results[i].append(packages[item])

        for i in range(n):
            if not is_selected[i]:
                cluster_results[k].append(packages[item])
        
        print(k)
        print(cluster_results)
        return cluster_results

    def tsp_path(self, packages, depot_x, depot_y):
        packages = [{"x": depot_x, "y": depot_y}] + packages
        n = len(packages)
        distance_matrix = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                distance_matrix[i][j] = distanceCalc(packages[i]["x"], packages[j]["x"], packages[i]["y"], packages[j]["y"], "Manhattan")
        cache = {}
        for i in range(1, n):
            curr = set((i,))
            curr_key = frozenset(curr)
            cache[curr_key] = {}
            cache[curr_key][i] = (distance_matrix[0][i], 0)

        for siz in range(2, n):
            for subset in itertools.combinations(range(1, n), siz):
                curr = set(subset)
                curr_key = frozenset(curr)
                cache[curr_key] = {}
                for elem in subset:
                    curr.remove(elem)
                    prev_key = frozenset(curr)
                    min_val = float('inf')
                    min_prev_elem = 0
                    for prev_elem in subset:
                        if prev_elem == elem:
                            continue
                        check = cache[prev_key][prev_elem][0] + distance_matrix[prev_elem][elem]
                        if check < min_val:
                            min_val = check
                            min_prev_elem = prev_elem
                    cache[curr_key][elem] = (min_val, min_prev_elem)
                    curr.add(elem)

        full_set = set(range(1,n))
        full_set_key = frozenset(full_set)

        min_val = float('inf')
        min_parent = 0
        for i in range(1, n):
            check = cache[full_set_key][i][0] + distance_matrix[i][0]
            if check < min_val:
                min_val = check
                min_parent = i
        
        parent = min_parent
        path_set = set(range(1, n))
        path = []
        sim_path = []
        for i in range(n-1):
            path.append(packages[parent])
            sim_path.append(packages[parent]["ID"])
            _, new_parent = cache[frozenset(path_set)][parent]
            path_set.remove(parent)
            parent = new_parent
                   
        return path


    def solve(self):
        packageRemain = True
        moveList = []
        while packageRemain:
            packageRemain = False
            for i in range(len(self.depots)):
                if len(self.depots[i]["Packages"]) > 0:
                    packageRemain = True
                    clusters = self.cluster(i)
                    index = 0
                    for cluster in clusters:
                        for elem in cluster:
                            moveList.append(("Load", self.depots[i]["Trucks"][index]["ID"], elem["ID"]))

                        path = self.tsp_path(cluster, self.depots[i]["x"], self.depots[i]["y"])
                        for node in path:
                            moveList.append(("Move", self.depots[i]["Trucks"][index]["ID"], node["x"], node["y"]))
                            moveList.append(("Drop", self.depots[i]["Trucks"][index]["ID"], node["ID"]))
                        moveList.append(("Move", self.depots[i]["Trucks"][index]["ID"], self.depots[i]["x"], self.depots[i]["y"]))
                        index += 1
                else:
                    min_depot = -1
                    for j in range(len(self.depots)):
                        if len(self.depots[j]["Packages"]) > 0:
                            if min_depot == -1:
                                min_depot = j
                            else:
                                distance_min = distanceCalc(self.depots[i]["x"], self.depots[min_depot]["x"], self.depots[i]["y"], self.depots[min_depot]["y"], "Manhattan")
                                distance = distanceCalc(self.depots[i]["x"], self.depots[j]["x"], self.depots[i]["y"], self.depots[j]["y"], "Manhattan")
                                if distance < distance_min:
                                    min_depot = j
                    if min_depot != -1:
                        trucks = self.depots[i]["Trucks"]
                        self.depots[i]["Trucks"] = []
                        self.depots[min_depot]["Trucks"] += trucks
                        for truck in trucks:
                            moveList.append(("Move", truck["ID"], self.depots[min_depot]["x"], self.depots[min_depot]["y"]))
        
        for move in moveList:
            print move
        # print(moveList)
        open(SAVE_FILE, 'w').close()
        with open(SAVE_FILE, 'w') as path_file:
            pickle.dump(moveList, path_file)

if __name__ == "__main__":
    args = parser.parse_args()
    LOAD_FILE = ROOT_PATH + "/" + args.LOAD_FILE
    solver = Solver()
    solver.solve()
    # rospy.init_node('solver')
    # pub_spawn = rospy.ServiceProxy("/gazebo/spawn_urdf_model",SpawnModel)
    # model_name="Turtlebot3_waffle"
    # model_xml=open("temp.urdf", "r").read()
    # reference_frame="world"
    # robot_namespace="A"
    # initial_pose = Pose()
    # initial_pose.position.x=0
    # initial_pose.position.y=0
    # initial_pose.position.z=0
    # initial_pose.orientation.x=0
    # initial_pose.orientation.y=0
    # initial_pose.orientation.z=0
    # initial_pose.orientation.w=0
    # result = pub_spawn(model_name=model_name, model_xml=model_xml, reference_frame=reference_frame, robot_namespace=robot_namespace, initial_pose=initial_pose)
    # pub_del = rospy.ServiceProxy("/gazebo/delete_model",DeleteModel)
    # pub_del(model_name)

        
