import math
import csv


def get_angle_of_waypoints(reference, target):
    # compute the two vectors formed by the points
    angle_degrees = math.degrees(math.atan2(target[1] - reference[1], target[0] - reference[0]))
    # adjust the angle to be between -180 and 180 degrees
    if angle_degrees > 180:
        angle_degrees -= 360
    elif angle_degrees < -180:
        angle_degrees += 360
    return angle_degrees


if __name__ == "__main__":
    waypoints = []
    with open("./src/navigation/src/waypoints.csv", newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            waypoints.append((float(row[0]), float(row[1])))

    directions = [0, 1, 2, 7, 8]

    for i in range(1, len(directions)):
        print(waypoints[directions[i-1]], waypoints[directions[i]])
        print(get_angle_of_waypoints(waypoints[directions[i-1]], waypoints[directions[i]]))