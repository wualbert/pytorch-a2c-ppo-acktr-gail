import sim.rigid_object_deprecated as object

import numpy as np
import pybullet as p
import unittest


class TestObject(unittest.TestCase):
    def setUp(self):
        p.connect(p.DIRECT)
        # p.connect(p.GUI)
        p.setRealTimeSimulation(0)
        # Set up a box
        self.p_W = np.asarray([0., 0., 0.25])
        self.obj = object.DefaultBoxObject(self.p_W)
        pos, orn = p.getBasePositionAndOrientation(self.obj.body_id)
        np.testing.assert_allclose(pos, self.p_W)
        np.testing.assert_allclose(p.getEulerFromQuaternion(orn), np.zeros(3))
        # This object has μ = 0.5, so the aperature should be tan⁻¹(0.5)
        self.assertAlmostEqual(self.obj.half_friction_cone_aperture,
                               np.arctan(0.5))
        p.stepSimulation()  # Timestep is 240Hz

    def test_compute_p_W_and_n_W(self):
        expected_x_W = np.asarray([2., 1., 0., -1., -2., -1., 0., 1., 2.])/2.
        expected_y_W = np.asarray([0., 1., 1., 1., 0., -1., -1., -1., 0.])/2.
        for p_B_W in [(0., 0., 0.), (0., 1., 0.), (1., 0., 0.),
                      (0., -1., 0.), (-1., 0., 0.)]:
            for yaw_W_init in np.linspace(0., np.pi*2, 7):
                q_B_W = p.getQuaternionFromEuler([0., 0., yaw_W_init])
                p.resetBasePositionAndOrientation(
                    self.obj.body_id, p_B_W, q_B_W)
                p.stepSimulation()  # Timestep is 240Hz
                for i, test_angle in enumerate(np.linspace(0., np.pi*2, 9)):
                    p_W, n_W = self.obj.compute_p_W_and_n_W(test_angle)
                    expected_p_W = \
                        np.dot(np.array(
                            p.getMatrixFromQuaternion(q_B_W)).reshape(3, 3),
                            np.asarray([expected_x_W[i],
                                        expected_y_W[i], 0.]))+p_B_W
                    np.testing.assert_allclose(p_W, expected_p_W, atol=1e-6)
                    if abs(expected_x_W[i]) == 1.:
                        expected_n_W = np.asarray(
                            [1., 0., 0.])*np.sign(expected_x_W[i])
                    else:
                        expected_n_W = np.asarray([0., 1., 0.]) * np.sign(
                            expected_y_W[i])
                    expected_n_W = np.dot(np.array(p.getMatrixFromQuaternion(
                        q_B_W)).reshape(3, 3), expected_n_W)
                    np.testing.assert_allclose(n_W, expected_n_W, atol=1e-6)
        # put the object back to the origin
        p.resetBasePositionAndOrientation(
            self.obj.body_id, np.zeros(3), np.array([0., 0., 0., 1.]))
        p.stepSimulation()  # Timestep is 240Hz

    def test_compute_friction_cone(self):
        for p_B_W in [(0., 0., 0.), (0., 1., 0.), (1., 0., 0.),
                      (0., -1., 0.), (-1., 0., 0.)]:
            for yaw_B_W in np.linspace(0., np.pi*2, 7):
                q_B_W = p.getQuaternionFromEuler([0., 0., yaw_B_W])
                p.resetBasePositionAndOrientation(
                    self.obj.body_id, p_B_W, q_B_W)
                p.stepSimulation()  # Timestep is 240Hz
                for i, test_angle in enumerate(np.linspace(0., np.pi*2, 9)):
                    expected_p_W, n_W = self.obj.compute_p_W_and_n_W(
                        test_angle)
                    c, s = np.cos(self.obj.half_friction_cone_aperture), \
                        np.sin(self.obj.half_friction_cone_aperture)
                    R = np.zeros((3, 3))
                    R[:2, :2] = np.array(((c, -s), (s, c)))
                    expected_b0_W = np.dot(R.T, n_W)
                    expected_b1_W = np.dot(R, n_W)
                    p_W, b0_W, b1_W = self.obj.compute_friction_cone(
                        test_angle)
                    np.testing.assert_allclose(p_W, expected_p_W, atol=1e-6)
                    np.testing.assert_allclose(b0_W, expected_b0_W, atol=1e-6)
                    np.testing.assert_allclose(b1_W, expected_b1_W, atol=1e-6)
                    np.testing.assert_allclose(np.linalg.norm(b0_W[:2]), 1.)
                    np.testing.assert_allclose(np.linalg.norm(b1_W[:2]), 1.)
        # put the object back to the origin
        p.resetBasePositionAndOrientation(
            self.obj.body_id, np.zeros(3), np.array([0., 0., 0., 1.]))
        p.stepSimulation()  # Timestep is 240Hz

    def test_compute_f_W_p(self):
        with self.assertRaises(AssertionError):
            self.obj.compute_f_W_p(0, -1, -1)
        for i, test_angle in enumerate(np.linspace(0., np.pi * 2, 9)):
            expected_p_W, expected_b0_W, expected_b1_W =\
                self.obj.compute_friction_cone(test_angle)
            for alpha_0 in np.linspace(0, 10, 11):
                for alpha_1 in np.linspace(0, 8, 11):
                    p_W, f_W_p = self.obj.compute_f_W_p(
                        test_angle, alpha_0, alpha_1)
                    expected_f_W_p = \
                        alpha_0*expected_b0_W+alpha_1*expected_b1_W
                    np.testing.assert_allclose(p_W, expected_p_W, atol=1e-6)
                    np.testing.assert_allclose(
                        f_W_p, expected_f_W_p, atol=1e-6)

    def tearDown(self):
        p.disconnect()


if __name__ == '__main__':
    unittest.main()
