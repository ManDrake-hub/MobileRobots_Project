import math
import csv
import matplotlib.pyplot as plt

class RobotController:
    def __init__(self, waypoints_path):
        self.waypoints = []
        self.x = []
        self.y = []
        self.robot_x = 0
        self.robot_y = 0
        self.robot_orientation = math.pi / 2  # 45 degrees
        self.avanti_x = []
        self.avanti_y = []
        self.indietro_x = []
        self.indietro_y = []
        self.destra_x = []
        self.destra_y = []
        self.sinistra_x = []
        self.sinistra_y = []
        # Read waypoints from CSV file
        with open(waypoints_path, newline='') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            for row in reader:
                self.waypoints.append((float(row[0]), float(row[1])))
        # Creazione del grafico a dispersione
        for x,y in self.waypoints:
            self.x.append(x)
            self.y.append(y)
        plt.scatter(self.x, self.y)
        plt.connect('button_press_event', self.on_click)
        plt.show()
        # Set initial robot position and orientation

    def on_click(self, event):
        if event.inaxes is not None:
            x, y = event.xdata, event.ydata
            print("You clicked on point ({}, {})".format(x, y))
            pose = "Inserisci la posa: "
            valore = input(pose)
            self.robot_x = x
            self.robot_y = y
            self.robot_orientation = int(valore)
            plt.close()

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

    def orientation_to_point(self, x, y):
        dx = x - self.robot_x
        dy = y - self.robot_y
        return math.atan2(dy, dx)

    def execute_command(self, command):
        if command == "avanti":
            self.avanti_x.append(self.robot_x)
            self.avanti_y.append(self.robot_y)
        elif command == "indietro":
            self.indietro_x.append(self.robot_x)
            self.indietro_y.append(self.robot_y)
        elif command == "destra":
            self.destra_x.append(self.robot_x)
            self.destra_y.append(self.robot_y)
        elif command == "sinistra":
            self.sinistra_x.append(self.robot_x)
            self.sinistra_y.append(self.robot_y)

    def find_closest_waypoint(self, command):
        closest_waypoint = None
        closest_distance = float('inf')
        for waypoint in self.waypoints:
            if command == "destra":
                if (waypoint[0] - self.robot_x) * math.sin(self.robot_orientation) - (waypoint[1] - self.robot_y) * math.cos(self.robot_orientation) < 0:
                    continue
            elif command == "sinistra":
                if (waypoint[0] - self.robot_x) * math.sin(self.robot_orientation) + (waypoint[1] - self.robot_y) * math.cos(self.robot_orientation) < 0:
                    continue
            elif command == "avanti":
                if (waypoint[0] - self.robot_x) * math.cos(self.robot_orientation) + (waypoint[1] - self.robot_y) * math.sin(self.robot_orientation) < 0:
                    continue
            elif command == "indietro":
                if (waypoint[0] - self.robot_x) * math.cos(self.robot_orientation) - (waypoint[1] - self.robot_y) * math.sin(self.robot_orientation) < 0:
                    continue
            d = self.distance(self.robot_x, self.robot_y, waypoint[0], waypoint[1])
            if d < closest_distance:
                closest_waypoint = waypoint
                closest_distance = d
        return closest_waypoint, closest_distance

    def navigate(self, command):
        self.execute_command(command)
        closest_waypoint, closest_distance = self.find_closest_waypoint(command)
        orientation_to_closest_waypoint = self.orientation_to_point(closest_waypoint[0], closest_waypoint[1])
        print("Closest waypoint:", closest_waypoint)
        print("Orientation to reach it:", orientation_to_closest_waypoint)
    
    def plot_waypoints(self):
        plt.scatter(self.x, self.y, label='Waypoints')
        plt.scatter(self.avanti_x, self.avanti_y, label='Avanti')
        plt.scatter(self.indietro_x, self.indietro_y, label='Indietro')
        plt.scatter(self.destra_x, self.destra_y, label='Destra')
        plt.scatter(self.sinistra_x, self.sinistra_y, label='Sinistra')
        plt.legend()
        plt.show()

robot = RobotController("/home/francesca/Scrivania/MobileRobots_Project/bot/src/navigation/src/waypoints.csv")  
robot.navigate("destra")
robot.navigate("avanti")
robot.navigate("sinistra")
robot.navigate("indietro")
robot.plot_waypoints()
# Esempio di array di punti x e y



