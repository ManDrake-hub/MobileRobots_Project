import math
import csv
import matplotlib.pyplot as plt

def on_click(event):
    if event.inaxes is not None:
        x, y = event.xdata, event.ydata
        print("You clicked on point ({}, {})".format(x, y))
        pose = "Inserisci la posa: "
        valore = input(pose)
        start_x = x
        start_y = y
        start_orient= int(valore)
        plt.close()
# Waypoint
waypoints = []
x_list = []
y_list = []
with open("/home/francesca/Scrivania/MobileRobots_Project/bot/src/navigation/src/waypoints.csv", newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                waypoints.append((float(row[0]), float(row[1])))
for x,y in waypoints:
            x_list.append(x)
            y_list.append(y)
plt.scatter(x_list, y_list)
plt.connect('button_press_event', on_click)
# Punto di partenza
start_x = -3.35
start_y = -9.5
start_orient = 0# in gradi

# Calcola l'angolo tra il punto di partenza e ogni waypoint
angles = []
for wp_x, wp_y in waypoints:
    angle = math.atan2(wp_y - start_y, wp_x - start_x) * 180 / math.pi
    angles.append(angle)

# Calcola la distanza euclidea tra il punto di partenza e ogni waypoint
distances = []
for wp_x, wp_y in waypoints:
    dist = math.sqrt((wp_x - start_x)**2 + (wp_y - start_y)**2)
    distances.append(dist)

# Trova i waypoint alla sinistra
left_waypoints = []
for i, angle in enumerate(angles):
    if angle >= start_orient + 90 and angle < start_orient + 180:
        left_waypoints.append((waypoints[i][0], waypoints[i][1]))

# Trova i waypoint alla destra
right_waypoints = []
for i, angle in enumerate(angles):
    if angle >= start_orient - 180 and angle < start_orient - 90:
        right_waypoints.append((waypoints[i][0], waypoints[i][1]))

# Trova i waypoint avanti
forward_waypoints = []
for i, angle in enumerate(angles):
    if angle >= start_orient and angle < start_orient + 90:
        forward_waypoints.append((waypoints[i][0], waypoints[i][1]))

# Trova i waypoint indietro
backward_waypoints = []
for i, angle in enumerate(angles):
    if angle >= start_orient - 90 and angle < start_orient:
        backward_waypoints.append((waypoints[i][0], waypoints[i][1]))

# Trova il waypoint più vicino a destra
if right_waypoints:
    right_distances = []
    for wp_x, wp_y in right_waypoints:
        dist = math.sqrt((wp_x - start_x)**2 + (wp_y - start_y)**2)
        right_distances.append(dist)
    min_dist = min(right_distances)
    min_dist_index = right_distances.index(min_dist)
    nearest_wp = right_waypoints[min_dist_index]
    plt.annotate("destra", (nearest_wp[0],nearest_wp[1]), textcoords="offset points", xytext=(0,10), ha='center')
    print("Waypoint più vicino a destra:", nearest_wp)
else:
    print("Nessun waypoint a destra")

# Trova il waypoint più vicino a sinistra
if left_waypoints:
    left_distances = []
    for wp_x, wp_y in left_waypoints:
        dist = math.sqrt((wp_x - start_x)**2 + (wp_y - start_y)**2)
        left_distances.append(dist)
    min_dist = min(left_distances)
    min_dist_index = left_distances.index(min_dist)
    nearest_wp = left_waypoints[min_dist_index]
    plt.annotate("sinistra", (nearest_wp[0],nearest_wp[1]), textcoords="offset points", xytext=(0,10), ha='center')
    print("Waypoint più vicino a sinistra:", nearest_wp)
else:
    print("Nessun waypoint a sinistra")

# Trova il waypoint più vicino avanti
if forward_waypoints:
    forward_distances = []
    for wp_x, wp_y in forward_waypoints:
        dist = math.sqrt((wp_x - start_x)**2 + (wp_y - start_y)**2)
        forward_distances.append(dist)
    min_dist = min(forward_distances)
    min_dist_index = forward_distances.index(min_dist)
    nearest_wp = forward_waypoints[min_dist_index]
    plt.annotate("avanti", (nearest_wp[0],nearest_wp[1]), textcoords="offset points", xytext=(0,10), ha='center')
    print("Waypoint più vicino avanti:", nearest_wp)
else:
    print("Nessun waypoint avanti")

# Trova il waypoint più vicino indietro
if backward_waypoints:
    backward_distances = []
    for wp_x, wp_y in backward_waypoints:
        dist = math.sqrt((wp_x - start_x)**2 + (wp_y - start_y)**2)
        backward_distances.append(dist)
    min_dist = min(backward_distances)
    min_dist_index = backward_distances.index(min_dist)
    nearest_wp = backward_waypoints[min_dist_index]
    plt.annotate("indietro", (nearest_wp[0],nearest_wp[1]), textcoords="offset points", xytext=(0,10), ha='center')
    print("Waypoint più vicino indietro:", nearest_wp)
else:
    print("Nessun waypoint indietro")
plt.show()
plt.waitforbuttonpress()
plt.close()
