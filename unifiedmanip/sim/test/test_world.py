import sim.world as world

import pybullet as p
import time
import unittest


class TestWorld(unittest.TestCase):
    def setUp(self):
        p.connect(p.DIRECT)
        # p.connect(p.GUI)
        p.setRealTimeSimulation(0)

    def test_constructor(self):
        world.World()

    def tearDown(self):
        p.disconnect()


class TestBoxWorld(unittest.TestCase):
    def setUp(self):
        p.connect(p.DIRECT)
        # p.connect(p.GUI)
        p.setRealTimeSimulation(0)

    def test_constructor(self):
        world.BoxWorld()

    def tearDown(self):
        p.disconnect()


if __name__ == '__main__':
    unittest.main()
