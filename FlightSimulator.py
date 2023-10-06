from math import tan, pi, cos, sin

class FlightSimulator:
    def __init__(self):
        self.g = 9.81
        self.X = [30]
        self.Z = [30]
        self.heading = [0]
        self.dt = 0.1

    def simulate(self, velocity, roll_degree):
        roll = roll_degree * pi / 180.0
        R = 0
        if roll != 0:
            R = velocity**2 / self.g / tan(roll)
        rate = 0
        if R != 0:
            rate = velocity / R
        self.heading.append(self.heading[-1] + rate * self.dt)
        self.X.append(self.X[-1] + velocity * cos(self.heading[-1]) * self.dt)
        self.Z.append(self.Z[-1] + velocity * sin(self.heading[-1]) * self.dt)

    def control(self, velocity_goal, roll_goal):
        if not (15 <= velocity_goal <= 25):
            raise ValueError("Скорость аппарата должна быть в диапазоне [15, 25]")
        if not (-20 <= roll_goal <= 20):
            raise ValueError("Угол крена аппарата должна находиться в диапазоне [-20, 20]")
        self.simulate(velocity_goal, roll_goal)

    def get_telemetry(self):
        return {
            "X": self.X[-1],
            "Z": self.Z[-1],
            "Heading": self.heading[-1]
        }
