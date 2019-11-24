import json
from math import degrees, atan2, sqrt
import operator
import numpy as np
import time



with open('piyush_environment.json') as inputFile:
    data = json.load(inputFile)

y_depot = data["depot"]["y"]
x_depot = data["depot"]["x"]

drop_place = {}
angles_list = [] 
for i in range (0,len(data["nodes"])):
	x = data["nodes"][i]["x"]
	y = data["nodes"][i]["y"]
	angle = degrees(atan2((y - y_depot),(x - x_depot)))
	angle_from_north = (90 - angle) % 360
	new = {str("customer_"+str(i)):{"angle":angle_from_north,"x":x,"y":y}}
	drop_place.update(new)

drop_place_list = drop_place.items()
sorted_drop_place =  sorted(drop_place_list, key = operator.itemgetter(1))
truck = [[('depot', {'y': y_depot, 'x': x_depot})]]
count = 0
j = 0
truck_num = data["vehicleCapacity"][j]
list_capacity = truck_num.values()
capacity = list_capacity[0]

def euclidean_distance(x1,y1,x2,y2):
	return abs(sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)))

for i in range(0,len(sorted_drop_place)+1):
	if count > capacity:
		if (j+1) < len(data["vehicleCapacity"]):
			j += 1
			truck.append([('depot', {'y': y_depot, 'x': x_depot})])
			truck_num = data["vehicleCapacity"][j]
			list_capacity = truck_num.values()
			capacity = list_capacity[0]
			count = 0
		else:
			break
	else:
		truck[j].append(sorted_drop_place.pop(0))
	count += 1

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
				dis_matrix[i][k] = euclidean_distance(x_depot,y_depot,truck[j][k][1]["x"],truck[j][k][1]["y"])
			
			elif k == 0:
				dis_matrix[i][k] = euclidean_distance(truck[j][i][1]["x"],truck[j][i][1]["y"],x_depot,y_depot)
			
			else:
				dis_matrix[i][k] = euclidean_distance(truck[j][i][1]["x"],truck[j][i][1]["y"],truck[j][k][1]["x"],truck[j][k][1]["y"])		
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
		while index_value in customers_visted:
			first[index_value] = 0
			a = filter(lambda a: a != 0, first)
			if not np.any(a):
				exit_time = 1
				route[j].append([truck[j][0]])
				break
			min_value = min(a)
			index_value =  first.index(min_value)
		if exit_time:
			break
		else:
			route.append([truck[j][index_value]])
			customers_visted.append(index_value)
			if len(route[j]) == len(truck[j]):
				flag = 0

	cluster_route.append(route)

for i in range (0, len(cluster_route[0])):
	print cluster_route[0][i][0][0]
print '\n'
for i in range (0, len(cluster_route[1])):
	print cluster_route[1][i][0][0]

