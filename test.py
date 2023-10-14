import unittest
from FieldSimulator import FieldSimulator
from solution import solver
import inspect

def get_current_function_name():
    return inspect.currentframe().f_back.f_back.f_code.co_name

class TestFieldSimulatorNewFormat(unittest.TestCase):

    def simulate(self, field_size, object_position, scan_radius=30, step_limit=2000):
        field_sim = FieldSimulator(field_size, object_position, scan_radius)
        solver_ex = solver(field_size)
        step = 0
        object_found = False
        while not object_found and step < step_limit:
            object_found = field_sim.control_drone(*solver_ex.solve(field_sim.drone_position()))
            step += 1
        print(get_current_function_name())
        print(field_sim.evaluate())
        field_sim.plot_field()
        self.assertTrue(object_found, "Object was not found within step limit.")
        return field_sim.evaluate()

    def test_large_field_object_corner(self):
        self.simulate((500, 500), (400, 400))

    def test_large_field_object_center(self):
        self.simulate((500, 500), (270, 250))

    def test_rectangle_field_object_edge(self):
        self.simulate((300, 833), (153, 694))

if __name__ == "__main__":
    unittest.main()
