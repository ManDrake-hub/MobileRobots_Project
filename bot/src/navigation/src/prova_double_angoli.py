import math
import csv
import matplotlib.pyplot as plt

def get_angle_of_waypoints(reference, target):
    # compute the two vectors formed by the points
    angle_degrees = math.degrees(math.atan2(target[1] - reference[1], target[0] - reference[0]))
    # adjust the angle to be between -180 and 180 degrees
    if angle_degrees > 180:
        angle_degrees -= 360
    elif angle_degrees < -180:
        angle_degrees += 360

    return angle_degrees

def on_click(event):
    if event.inaxes is not None:
        x, y = event.xdata, event.ydata
        print("You clicked on point ({}, {})".format(x, y))
        plt.close()

if __name__ == "__main__":
    waypoints = []
    x_list=[]
    y_list=[]
    avanti = []
    indietro = []
    sinistra = []
    destra = []

    with open("/home/francesca/Scrivania/MobileRobots_Project/bot/src/navigation/src/waypoints.csv", newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            waypoints.append((float(row[0]), float(row[1])))
    for x,y in waypoints:
            x_list.append(x)
            y_list.append(y)
    plt.scatter(x_list, y_list)
    plt.connect('button_press_event', on_click)

    #for i in range(1, len(directions)):
    #    print(waypoints[directions[i-1]], waypoints[directions[i]])
    actual_waypoint = waypoints[1]
    for i in range(len(waypoints)):
        print(actual_waypoint,waypoints[i])
        print(get_angle_of_waypoints(actual_waypoint,waypoints[i]))
        if -45 <= get_angle_of_waypoints(actual_waypoint,waypoints[i]) <= 45: 
            print("avanti")
            avanti.append(waypoints[i])
            plt.annotate("avanti", (waypoints[i][0],waypoints[i][1]), textcoords="offset points", xytext=(0,10), ha='center')
        if -180 <= get_angle_of_waypoints(actual_waypoint,waypoints[i]) <= -135 or 135 <= get_angle_of_waypoints(actual_waypoint,waypoints[i]) <= 180 : 
            print("indietro")
            indietro.append(waypoints[i])
            plt.annotate("indietro", (waypoints[i][0],waypoints[i][1]), textcoords="offset points", xytext=(0,10), ha='center')
        if 45 <= get_angle_of_waypoints(actual_waypoint,waypoints[i]) <= 135: 
            print("sinistra")
            sinistra.append(waypoints[i])
            plt.annotate("sinistra", (waypoints[i][0],waypoints[i][1]), textcoords="offset points", xytext=(0,10), ha='center')
        if -135 <= get_angle_of_waypoints(actual_waypoint,waypoints[i]) <= -45: 
            print("destra")
            destra.append(waypoints[i])
            plt.annotate("destra", (waypoints[i][0],waypoints[i][1]), textcoords="offset points", xytext=(0,10), ha='center')

plt.show()
plt.waitforbuttonpress()
plt.close()
