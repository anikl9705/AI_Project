    def drop_place_angles(self, x_depot, y_depot,depot_no):
        i = 0
        drop_place = {}
        
        for package_no in range(0, len(self.depots[depot_no]['Packages'])):
            i += 1
            x = self.depots[depot_no]['Packages'][package_no]['x']
            y = self.depots[depot_no]['Packages'][package_no]['y']
            pack_size = self.depots[depot_no]['Packages'][package_no]['Size']
            pack_id = self.depots[depot_no]['Packages'][package_no]['ID']
            angle = degrees(atan2((y - y_depot),(x - x_depot)))
            angle_from_north = (90 - angle) % 360
            new = {str("customer_"+str(i)):{"angle":angle_from_north,"x":x,"y":y, "pack_size":pack_size,"pack_id":pack_id}}
            drop_place.update(new) 
        
        drop_place_list = drop_place.items()
        sorted_drop_place =  sorted(drop_place_list, key = operator.itemgetter(1))
        return sorted_drop_place

    def solver_piyush(self):
        moveList = []
        for depot_no in range(0, len(self.depots)):
            x_depot = self.depots[depot_no]['x']
            y_depot = self.depots[depot_no]['y']
            list_with_angles = self.drop_place_angles(x_depot,y_depot,depot_no)
            cluster = [[('depot', {'y': y_depot, 'x': x_depot})]]
            index = -1
            flag_1 = 1

            while list_with_angles:
                for truck_no in range(0,len(self.depots[depot_no]["Trucks"])):
                    capacity = self.depots[depot_no]["Trucks"][truck_no]["Capacity"]
                    truck_id = self.depots[depot_no]["Trucks"][truck_no]["ID"]
                    size = 0
                    size_index = 1
                    index += 1
                    if index != 0:
                        cluster.append([('depot', {'y': y_depot, 'x': x_depot})])
                    flag = 1
                    while flag:
                        if not list_with_angles:
                            flag_1 = 0
                            break
                        out = list_with_angles.pop(0) 
                        out_list = list(out)   
                        out_list.append(truck_id)
                        out = tuple(out_list)
                        cluster[index].append(out)
                        size += cluster[index][size_index][1]['pack_size']
                        moveList.append(("Load", truck_id, cluster[index][size_index][1]['pack_id']))
                        if size >= capacity:
                            flag = 0
                        size_index +=1
                
                if flag_1 == 0:
                    break
            self.distance_matrix(cluster,x_depot,y_depot,moveList)
        
        print moveList
        with open(SAVE_FILE, 'wb') as path_file:
            pickle.dump(moveList, path_file)
            
    def distance_matrix(self,truck,x_depot,y_depot,moveList):
        matrix = []
        
        for j in range (0,len(truck)):
            rows = len(truck[j])
            cloumns = len(truck[j])
            dis_matrix = [[0 for p in range(cloumns)] for q in range(rows)] 
            
            for i in range(0,len(truck[j])):
                
                for k in range(0,len(truck[j])):
                    if i == k:
                        continue
                    
                    if i == 0: 
                        dis_matrix[i][k] = distanceCalc(x_depot,truck[j][k][1]["x"],y_depot,truck[j][k][1]["y"],"Manhattan")
                    
                    elif k == 0:
                        dis_matrix[i][k] = distanceCalc(truck[j][i][1]["x"],x_depot,truck[j][i][1]["y"],y_depot,"Manhattan")
                    
                    else:
                        dis_matrix[i][k] = distanceCalc(truck[j][i][1]["x"],truck[j][k][1]["x"],truck[j][i][1]["y"],truck[j][k][1]["y"],"Manhattan")      
            matrix.append(dis_matrix)

        cluster_route = []
        
        for j in range(0,len(matrix)):
            route = []
            route.append([truck[j][0]])
            customers_visted = []
            index_value = 0
            flag = 1
            exit_time = 0
            
            while flag:

                first = matrix[j][index_value]
                a = filter(lambda a: a != 0, first)
                min_value = min(a)
                index_value =  first.index(min_value)
                
                if index_value == 0:
                    first[index_value] = 0
                    a = filter(lambda a: a != 0, first)
                    
                    if not np.any(a):
                        route.append([truck[j][0]])
                        moveList.append(("Move", truck_id, truck[j][0][1]['x'], truck[j][0][1]['y']))
                        break
                    
                    min_value = min(a)
                    index_value =  first.index(min_value)
                
                while index_value in customers_visted:
                    first[index_value] = 0
                    a = filter(lambda a: a != 0, first)
                    
                    if not np.any(a):
                        exit_time = 1
                        route.append([truck[j][0]])
                        moveList.append(("Move", truck_id, truck[j][0][1]['x'], truck[j][0][1]['y']))
                        break
                    
                    min_value = min(a)
                    index_value =  first.index(min_value)
                    
                    if index_value == 0:
                        first[index_value] = 0
                        a = filter(lambda a: a != 0, first)
                        
                        if not np.any(a):
                            exit_time = 1
                            route.append([truck[j][0]])
                            moveList.append(("Move", truck_id, truck[j][0][1]['x'], truck[j][0][1]['y']))
                            break

                        min_value = min(a)
                        index_value =  first.index(min_value)
                
                if exit_time:
                    break
                else:
                    route.append([truck[j][index_value]])
                    truck_id = truck[j][index_value][2]
                    moveList.append(("Move", truck_id, truck[j][index_value][1]['x'], truck[j][index_value][1]['y']))
                    moveList.append(("Drop", truck_id, truck[j][index_value][1]['pack_id']))

                    customers_visted.append(index_value)
                    if len(route[j]) == len(truck[j]):
                        flag = 0
            cluster_route.append(route)
        return