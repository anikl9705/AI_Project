def get_path(self, init_state, goal_locations):
        """
        This method searches for a path from init_state to one of the possible goal_locations

        :param init_state: Current state of robot
        :type init_state: State
        :param goal_locations: list of target locations to search the path e.g. [(x1, y1), (x2, y2), ..]. This is important if there are multiple books of a subject.
        :type goal_locations: list(State)

        :returns: 
            .. hlist::
                :columns: 1

                * **action_list**: list of actions to execute to go from source to target
                * **final_state**: target state that is reached (will be one of goal_locations)
                * **goal_reached**: True/False indicating if one of the goal_locations was reached
        
        """
        final_state = None
        goal_states = self.build_goal_states(goal_locations)
        goal_reached = False
        for goal_state in goal_states: #search for any of the load locations
            possible_actions = self.helper.get_actions()
            action_list = []
            
            state_queue = []
            heapq.heappush(state_queue, (self.get_manhattan_distance(init_state, goal_state), 0,(init_state, []),0))
            visited = []
            state_cost = {}
            insert_order = 0

            while len(state_queue) > 0:
                _,_,current_state,current_actions,current_cost=heapq.heappop(state_queue)
                # top_item = heapq.heappop(state_queue)
                # current_cost = top_item[0]
                # current_state = top_item[2][0]
                # current_actions = top_item[2][1]
                
                if(current_state in visited):
                    continue
                
                if(self.is_goal_state(current_state, goal_state)):
                    goal_reached = True
                    break
                    return action_list

                visited.append(current_state)
                for action in possible_actions:
                    nextstate, cost = self.helper.get_successor(current_state, action)
                    n_cost = self.get_manhattan_distance(nextstate, goal_state) # manhattan distance heuristc
                    key = (nextstate.x, nextstate.y, nextstate.orientation)
                    if(nextstate.x == -1 and nextstate.y == -1):
                        continue
                    # if a new state is found then add to queue
                    if(nextstate not in visited and key not in state_cost.keys()):
                        heapq.heappush(state_queue, (n_cost+cost+current_cost, insert_order, (nextstate, current_actions + [action]),cost+current_cost))
                        insert_order += 1
                        state_cost[key] = cost
                    
            if(self.is_goal_state(current_state, goal_state)):
                action_list = current_actions
                final_state = current_state
                goal_reached = True
                break

        return action_list, final_state, goal_reached

# ------------------------------------------------ #
# DO NOT MODIFY THE CODE BELOW