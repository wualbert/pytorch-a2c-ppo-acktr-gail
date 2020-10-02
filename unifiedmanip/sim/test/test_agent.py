import sim.agent as agent
import sim.world as world

import numpy as np
import pybullet as p
import time  # noqa
import unittest


class TestCarrotAndStickAgent(unittest.TestCase):
    def setUp(self):
        # p.connect(p.GUI)
        p.connect(p.DIRECT)
        self.box_world = world.BoxWorld()

    def test_spinning_in_place(self):
        agnt_0 = agent.CarrotAndStickAgent(
            list(self.box_world.object_dict.values())[0])
        agnt_1 = agent.CarrotAndStickAgent(
            list(self.box_world.object_dict.values())[0], phi=np.pi)
        self.box_world.agent_list.extend([agnt_0, agnt_1])
        for i in range(1000):
            self.box_world.step_simulation()
        pos, orn = p.getBasePositionAndOrientation(
            list(self.box_world.object_dict.keys())[0])
        np.testing.assert_allclose(pos, np.asarray([0., 0., 0.25]), atol=1e-4)

    def test_pushing(self):
        obj = list(self.box_world.object_dict.values())[0]
        agnt_0 = agent.CarrotAndStickAgent(obj, alpha_0=2, alpha_1=2)
        obj_mu = p.getDynamicsInfo(obj.body_id, -1)[1]
        # 1 + tan² = sec²
        # force = 2*1*cos
        force = 2.*np.sqrt(1./(1.+obj_mu**2))*2
        # This should yield approximately 1.78885438 Newton
        self.box_world.agent_list.extend([agnt_0])
        pos, orn = p.getBasePositionAndOrientation(
            list(self.box_world.object_dict.keys())[0])
        p.setTimeStep(1e-4)
        for i in range(5000):
            # Execute for 0.5 second
            self.box_world.step_simulation()
        pos, orn = p.getBasePositionAndOrientation(
            list(self.box_world.object_dict.keys())[0])
        # Δx = 0.5⋅a⋅t²
        # Note that the tolerance is really lenient here. The analytical
        # answer is not always identical to the one from the physics engine
        np.testing.assert_allclose(pos, np.asarray(
            [0.5*force*0.5**2, 0., 0.25]), atol=1e-2)
        p.setTimeStep(1./240.)

    def tearDown(self):
        p.disconnect()


if __name__ == '__main__':
    unittest.main()
