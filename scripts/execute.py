#!/usr/bin/env python

from group_4.srv import *
from decimal import Decimal
import rospy
import os
import yaml
import pickle
import argparse
import heapq
from std_msgs.msg import String
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState, ModelStates

ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
parser = argparse.ArgumentParser()
parser.add_argument('-p', help='for specifying plan file', metavar='5', action='store', dest='plan_file', default='plan_path.pkl', type=str)
parser.add_argument('-f', help='for specifying environment file', metavar='5', action='store', dest='env_file', default='temp_env.json', type=str)

pkl_file = open(ROOT_PATH + "/weights.pkl", "r")
weight_matrix = pickle.load(pkl_file)

orientations = {
    "NORTH": {
        "x": 0.000000000000, 
        "y": 0.000000000000,
        "z": 0.70710678118,
        "w": 0.70710678118
    }, 
    "EAST": {
        "x": 0.00000000000, 
        "y": 0.00000000000,
        "z": 0.00000000000,
        "w": 0.99999983
    },
    "SOUTH": {
        "x": 0.00000000000, 
        "y": 0.00000000000,
        "z": -0.70710678118,
        "w": 0.70710678118
    },
    "WEST": {
        "x": 0.00000000000, 
        "y": 0.00000000000,
        "z": 0.99999983,
        "w": 0.00000000000
    }
}

class State:
    def __init__(self, x, y, orientation):
        self.x = x
        self.y = y
        self.orientation = orientation

def get_state_from_dict(truck):
    return State(truck['x'], truck['y'], truck['orientation'])

def update_truck_from_state(truck, state):
    truck['x'] = state.x
    truck['y'] = state.y
    truck['orientation'] = state.orientation
    return truck

class ActionManager:
    def __init__(self, depots):
        self.depots = depots
        self.trucks = {}
        for depot in depots:
            for truck in depot["Trucks"]:
                temp_truck = {
                    'x': depot['x'],
                    'y': depot['y'],
                    'orientation': "EAST",
                    'Packages': set()
                }
                self.trucks[truck["ID"]] = temp_truck
        print(self.trucks)
        self.current_truck = None
        self.cost_multiplier = {
            "MoveF": 0.5,
            "TurnCW": 0.75,
            "TurnCCW": 1
        }
        self.direction_list = ["NORTH", "EAST", "SOUTH", "WEST"]
        self.action_publisher = rospy.Publisher("/actions", String, queue_size=10)
        self.status_publisher = rospy.Publisher("/status", String, queue_size=10)
        self.deleter = rospy.ServiceProxy("/gazebo/delete_model",DeleteModel)
        self.spawner = rospy.ServiceProxy("/gazebo/spawn_urdf_model",SpawnModel)
        self.shifter = rospy.Publisher("/gazebo/set_model_state",ModelState,queue_size = 10)

    def check_edge(self, x1, x2, y1, y2):
        rospy.wait_for_service('check_is_edge')
        try:
            check_is_edge = rospy.ServiceProxy('check_is_edge',CheckEdge)
            if x1 <= x2 and y1 <= y2:
                result = check_is_edge(x1,y1,x2,y2)
            else:
                result = check_is_edge(x2,y2,x1,y1)
            return result.value == 1
        except rospy.ServiceException,e:
            print "Sevice call failed: %s"%e

    def get_current_truck(self):
        return self.current_truck

    def get_current_state(self):
        if self.current_truck is None:
            return None
        return get_state_from_dict(self.trucks[self.current_truck])

    def spawn_truck(self, truck_id):
        model_name="turtlebot3_waffle"
        truck = self.trucks[truck_id]
        state = get_state_from_dict(truck)
        print("############## SPAWN ########")
        print(truck)
        # if self.current_truck != None:
        model_state_msg = ModelState()
        model_state_msg.model_name = model_name
        model_state_msg.pose.position.x = state.x
        model_state_msg.pose.position.y = state.y
        model_state_msg.pose.position.z = 0
        model_state_msg.pose.orientation.x=orientations[state.orientation]["x"]
        model_state_msg.pose.orientation.y=orientations[state.orientation]["y"]
        model_state_msg.pose.orientation.z=orientations[state.orientation]["z"]
        model_state_msg.pose.orientation.w=orientations[state.orientation]["w"]
        self.shifter.publish(model_state_msg)
        # else:
        #     model_xml=open(ROOT_PATH + "/" + "temp.urdf", "r").read()
        #     reference_frame="world"
        #     robot_namespace = "rob"
        #     initial_pose = Pose()
        #     initial_pose.position.x=Decimal(state.x)
        #     initial_pose.position.y=Decimal(state.y)
        #     initial_pose.position.z=0
        #     initial_pose.orientation.x=orientations[state.orientation]["x"]
        #     initial_pose.orientation.y=orientations[state.orientation]["y"]
        #     initial_pose.orientation.z=orientations[state.orientation]["z"]
        #     initial_pose.orientation.w=orientations[state.orientation]["w"]
        #     rospy.wait_for_service('gazebo/spawn_urdf_model')
        #     self.spawner(model_name=model_name, model_xml=model_xml, reference_frame=reference_frame, robot_namespace=robot_namespace, initial_pose=initial_pose)
        self.current_truck = truck_id

    def execute_drop(self, truck_id, package_id):
        if package_id in self.trucks[truck_id]["Packages"]:
            state = self.get_current_state()
            print("Dropping at %f %f" %(state.x, state.y))
            package_no = int(package_id.split("_")[1])
            model_name = "Mailbox_{0}".format(package_no+1)
            rospy.wait_for_service('gazebo/delete_model')
            self.deleter(model_name)
        else:
            print("Fail")

    def execute_load(self, truck_id, package_id):
        self.trucks[truck_id]["Packages"].add(package_id)

    def check_action(self, current_state, action):
        x1 = current_state.x
        y1 = current_state.y
        orientation = current_state.orientation
        next_state = State(x1, y1, orientation)
        cost = self.cost_multiplier["MoveF"]

        if action == "MoveF":
            # Get new location
            if "EAST" == orientation:
                x2 = x1 + 0.5
                y2 = y1
                x_int = int(2*min(x1, x2))
                y_int = int(2*y1)
                cost *= weight_matrix[x_int][y_int][1]
            elif "WEST" == orientation:
                x2 = x1 - 0.5
                y2 = y1
                x_int = int(2*min(x1, x2))
                y_int = int(2*y1)
                cost *= weight_matrix[x_int][y_int][1]
            elif "NORTH" == orientation:
                x2 = x1
                y2 = y1 + 0.5
                x_int = int(2*x1)
                y_int = int(2*min(y1,y2))
                cost *= weight_matrix[x_int][y_int][0]
            else:
                x2 = x1
                y2 = y1 - 0.5
                x_int = int(2*x1)
                y_int = int(2*min(y1,y2))
                cost *= weight_matrix[x_int][y_int][0]
            if self.check_edge(x1,y1,x2,y2):
                next_state.x = x2
                next_state.y = y2
                return True, next_state, cost
            else:
                return False, next_state, cost
        elif action == "TurnCW":
            next_state.orientation = self.direction_list[(self.direction_list.index(current_state.orientation) + 1)%4]
            return True, next_state, self.cost_multiplier["TurnCW"]
        else:
            next_state.orientation = self.direction_list[(self.direction_list.index(current_state.orientation) - 1)%4]
            return True, next_state, self.cost_multiplier["TurnCW"]

    def execute_move(self, action):
        if action == "MoveF":
            return self.execute_MoveF(self.current_truck)
        elif action == "TurnCW":
            return self.execute_TurnCW(self.current_truck)
        else:
            return self.execute_TurnCCW(self.current_truck)

    # Modified from Action Server
    def execute_MoveF(self, truck_id):
        current_state = get_state_from_dict(self.trucks[truck_id])
        x1 = current_state.x
        y1 = current_state.y
        orientation = current_state.orientation

        # Get new location
        cost = self.cost_multiplier["MoveF"]
        if "EAST" == orientation:
            x2 = x1 + 0.5
            y2 = y1
            x_int = int(2*min(x1, x2))
            y_int = int(2*y1)
            cost *= weight_matrix[x_int][y_int]
        elif "WEST" == orientation:
            x2 = x1 - 0.5
            y2 = y1
            x_int = int(2*min(x1, x2))
            y_int = int(2*y1)
            cost *= weight_matrix[x_int][y_int]
        elif "NORTH" == orientation:
            x2 = x1
            y2 = y1 + 0.5
            x_int = int(2*x1)
            y_int = int(2*min(y1,y2))
            cost *= weight_matrix[x_int][y_int]
        else:
            x2 = x1
            y2 = y1 - 0.5
            x_int = int(2*x1)
            y_int = int(2*min(y1,y2))
            cost *= weight_matrix[x_int][y_int]

        # Check if that edge isn't blocked
        if self.check_edge(x1,y1,x2,y2):
            action_str = "MoveF"

            # Make bot move if simulating in gazebo
            # if simulation:
            self.action_publisher.publish(String(data=action_str))
            rospy.wait_for_message("/status",String)
            
            # Update State
            current_state.x = x2
            current_state.y = y2
            self.trucks[truck_id] = update_truck_from_state(self.trucks[truck_id], current_state)

            return True, cost
        else:
            return False, cost

    
    def execute_TurnCW(self, truck_id):
        current_state = get_state_from_dict(self.trucks[truck_id])
        cost = self.cost_multiplier["TurnCW"]
        
        # Make bot move if simulating in gazebo
        # if simulation:
        action_str = "TurnCW"
        self.action_publisher.publish(String(data=action_str))
        rospy.wait_for_message("/status",String)
        
        # Update state
        current_orientation = current_state.orientation
        new_orientation = self.direction_list[(self.direction_list.index(current_orientation) + 1)%4]
        current_state.orientation = new_orientation
        self.trucks[truck_id] = update_truck_from_state(self.trucks[truck_id], current_state)

        return True, cost


    def execute_TurnCCW(self, truck_id):
        current_state = get_state_from_dict(self.trucks[truck_id])
        cost = self.cost_multiplier["TurnCCW"]
        
        # Make bot move if simulating in gazebo
        # if simulation:
        action_str = "TurnCCW"
        self.action_publisher.publish(String(data=action_str))
        rospy.wait_for_message("/status",String)
        
        # Update state
        current_orientation = current_state.orientation
        new_orientation = self.direction_list[(self.direction_list.index(current_orientation) - 1)%4]
        current_state.orientation = new_orientation
        self.trucks[truck_id] = update_truck_from_state(self.trucks[truck_id], current_state)

        return True, cost

class Executor:
    def __init__(self, file_name, plan_file_name):
        rospy.init_node('simulator')
        self.plan_file_name = plan_file_name
        self.object_dict = yaml.load(open(file_name, "r"))["Depots"]
        self.helper = ActionManager(self.object_dict)
        self.action_queue = self.refine(plan_file_name, self.object_dict)
        for action_queue_elem in self.action_queue:
            print(action_queue_elem)
        self.execute_actions(self.action_queue)
        rospy.spin()

    def execute_actions(self, action_queue):
        for action in action_queue:
            if action[1] != self.helper.get_current_truck():
                self.helper.spawn_truck(action[1])
            action_type = action[0]
            if action_type == "Load":
                print("LOAD %s %s" %(action[1], action[2]))
                self.helper.execute_load(action[1], action[2])
            elif action_type == "Drop":
                print("DROP %s %s" %(action[1], action[2]))
                self.helper.execute_drop(action[1], action[2])
            else:
                print("MOVE %s" %(action[1]))
                path = action[2]
                print(path)
                for move in path:
                    self.helper.execute_move(move)

    def refine(self, plan_file_name, depots):
        trucks = {}
        for depot in depots:
            for truck in depot["Trucks"]:
                temp_truck = {
                    'x': depot['x'],
                    'y': depot['y'],
                    'orientation': "EAST",
                    'Packages': set()
                }
                trucks[truck["ID"]] = temp_truck
        plan_file = open(plan_file_name, "r")
        higher_actions = pickle.load(plan_file)
        action_queue = []
        for i in range(0, len(higher_actions)):
            action = higher_actions[i]
            action_type = action[0]
            if action_type == "Load" or action_type == "Drop":
                action_queue.append(action)
            else:
                current_state = get_state_from_dict(trucks[action[1]])
                goal_state = State(action[2], action[3], "EAST")
                path, final_state, _ = self.get_path(current_state, goal_state)
                trucks[action[1]] = update_truck_from_state(truck, final_state)
                action_queue.append(("Move", action[1], path))
        return action_queue

    def is_goal_state(self, current_state, goal_state):
        if(current_state.x == goal_state.x and current_state.y == goal_state.y):
            return True
        return False

    def get_manhattan_distance(self, state1, state2):
        return abs(state1.x - state2.x) + abs(state1.y - state2.y)

    def get_path(self, init_state, goal_state):
        final_state = None
        goal_reached = False
        possible_actions = ['MoveF', 'TurnCW', 'TurnCCW']
        action_list = []
        
        state_queue = []
        heapq.heappush(state_queue, (self.get_manhattan_distance(init_state, goal_state), 0,(init_state, []),0))
        visited = []
        state_cost = {}
        insert_order = 0

        while len(state_queue) > 0:
            _,_,current,current_cost=heapq.heappop(state_queue)
            current_state, current_actions = current
            # top_item = heapq.heappop(state_queue)
            # current_cost = top_item[0]
            # current_state = top_item[2][0]
            # current_actions = top_item[2][1]
            
            if current_state in visited:
                continue
            
            if self.is_goal_state(current_state, goal_state):
                goal_reached = True
                break

            visited.append(current_state)
            for action in possible_actions:
                success, next_state, cost = self.helper.check_action(current_state, action)
                if not success:
                    continue
                n_cost = self.get_manhattan_distance(next_state, goal_state) # manhattan distance heuristc
                key = (next_state.x, next_state.y, next_state.orientation)
                if(next_state.x == -1 and next_state.y == -1):
                    continue
                # if a new state is found then add to queue
                if(next_state not in visited and key not in state_cost.keys()):
                    heapq.heappush(state_queue, (n_cost+cost+current_cost, insert_order, (next_state, current_actions + [action]),cost+current_cost))
                    insert_order += 1
                    state_cost[key] = cost
        
        if self.is_goal_state(current_state, goal_state):
            action_list = current_actions
            final_state = current_state
            goal_reached = True

        return action_list, final_state, goal_reached


if __name__ == "__main__":
    args = parser.parse_args()
    plan_file = ROOT_PATH + "/" + args.plan_file
    env_file = ROOT_PATH + "/" + args.env_file
    simulator = Executor(env_file, plan_file)

