from FlightSimulator import FlightSimulator
from math import pi, sqrt
from solution import solver

import matplotlib.pyplot as plt

class FieldSimulator:
    def __init__(self, field_size, object_position, visibility_radius, resolution=1):
        self.field_size = field_size
        self.object_position = object_position
        self.visibility_radius = visibility_radius
        self.flight_sim = FlightSimulator()
        self.time_spent = 0
        self.scans = 0
        self.resolution = resolution
        self.scanned_pixels = set()
        self.object_found = False

    def drone_position(self):
        return self.flight_sim.get_telemetry()

    def scan(self):
        self.scans += 1
        drone_pos = self.drone_position()
        z, x = drone_pos["Z"], drone_pos["X"]

        for j in range(int(z - self.visibility_radius), int(z + self.visibility_radius), self.resolution):
            for i in range(int(x - self.visibility_radius), int(x + self.visibility_radius), self.resolution):
                if self._distance(x, z, i, j) <= self.visibility_radius:
                    if 0 < j < self.field_size[0] and 0 < i < self.field_size[1]:
                        self.scanned_pixels.add((j, i))

        if self._distance(z, x, self.object_position[0], self.object_position[1]) <= self.visibility_radius:
            self.object_found = True
            return True

        return False

    def _distance(self, z1, x1, z2, x2):
        return sqrt((z2 - z1)**2 + (x2 - x1)**2)

    def evaluate(self):
        total_pixels = (self.field_size[0] // self.resolution) * (self.field_size[1] // self.resolution)
        scanned_percentage = (len(self.scanned_pixels) / total_pixels) * 100
        return {
            "Time Spent": f"{self.time_spent:.1f}",
            "Scanned Percentage": f"{scanned_percentage:.3f}",
            "Scan counter": f"{self.scans}",
            "Scan effectiveness": f"{scanned_percentage/self.scans:.3f}",
            "Object found": f"{self.object_found}"
        }

    def control_drone(self, velocity_goal, roll_goal, scan=False):
        self.flight_sim.control(velocity_goal, roll_goal)
        self.time_spent += 0.1
        if scan:
            return self.scan()
        return False

    def plot_field(self):
        field = [[0 for i in range(self.field_size[0])] for j in range(self.field_size[1])]

        for z, x in self.scanned_pixels:
            if 0 <= z < self.field_size[0] and 0 <= x < self.field_size[1]:
                field[x][z] = 1

        plt.imshow(field, cmap='gray', origin='lower')
        plt.colorbar(label='Сканированная часть')
        plt.title('Просканированная часть поля')
        plt.xlabel('Координата Z')
        plt.ylabel('Координата X')
        plt.scatter(self.object_position[0],self.object_position[1], c='red', marker='x', label="Объект")
        plt.legend()
        plt.show()

if __name__ == "__main__":
    field_size = (500, 500)
    scan_radius = 30
    field_sim = FieldSimulator(field_size, (415,300), scan_radius)
    solver_ex = solver(field_size)
    step = 0
    step_limit = 2000
    object_found = False
    while not(object_found) and step < step_limit:
        object_found = field_sim.control_drone(*solver_ex.solve(field_sim.drone_position()))
        step += 1

    field_sim.plot_field()
    print(field_sim.evaluate())
