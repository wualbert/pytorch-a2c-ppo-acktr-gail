import numpy as np
import pybullet as p

import os


class Object:
    def __init__(self, object_file, p_W_init=None,
                 q_W_init=None, **kwargs):
        self.object_file = object_file
        if self.object_file.split('.')[-1] == "sdf":
            self.body_id = p.loadSDF(self.object_file)[0]
        elif self.object_file.split('.')[-1] == "urdf":
            self.body_id = p.loadURDF(self.object_file)
        else:
            raise NotImplementedError
        # Friction coefficient
        self.mu = p.getDynamicsInfo(self.body_id, -1)[1]
        print('mu', self.mu)
        self.half_friction_cone_aperture = np.arctan(self.mu)
        assert (0 <= self.half_friction_cone_aperture <= np.pi/2)
        if p_W_init is not None:
            assert len(p_W_init) == 3
        else:
            p_W_init = np.zeros(3)
        if q_W_init is not None:
            assert len(q_W_init) == 4
        else:
            # [x,y,z,w]
            q_W_init = np.zeros(4)
            q_W_init[-1] = 1.
        p.resetBasePositionAndOrientation(self.body_id, p_W_init, q_W_init)

    def compute_p_W_and_n_W(self, phi_B):
        """
        Given a planar angle in the object frame B, perform ray cast from the
        origin of the B frame and find the corresponding surface normal and
        ray impact position in the world frame W.
        FIXME(wualbert): We don't care about z as we are in 2D.
                         This may cause problems later.
        @param phi_B: The yaw of the contact point in the B frame.
        @return: (p_W, n_W). p_W is the 3D contact point on the object.
        n_W is the 3D outward-facing unit normal vector at the contact point.
        Both of these are in the world frame.
        """
        # Get the pose of the object frame origin in world frame
        p_B_W, q_B_W = p.getBasePositionAndOrientation(self.body_id)
        e_B_W = p.getEulerFromQuaternion(q_B_W)
        # In 2D, only the yaw should be nonzero
        np.testing.assert_allclose(e_B_W[:2], np.zeros(2), atol=1e-4)
        # add phi_B
        e_phi_W = e_B_W + np.asarray([0., 0., phi_B])
        e_phi_W %= 2*np.pi
        q_phi_W = p.getQuaternionFromEuler(e_phi_W)
        # Construct the unit ray vector
        # Compute the ray to cast
        ray_direction_W = np.dot(
            np.array(p.getMatrixFromQuaternion(q_phi_W)).reshape(3, 3),
            np.asarray([1., 0., 0.]))
        # FIXME(wualbert): the ray shouldn't be of arbitrary length
        ray_start_W = p_B_W + ray_direction_W*10.
        ray_end_W = p_B_W - ray_direction_W * 10.
        ans = p.rayTest(ray_start_W, ray_end_W)
        for a in ans:
            if a[0] == self.body_id:
                # FIXME(wualbert): a cleaner way to do this
                # We only care about the normal of this particular object
                return a[3:]
        raise AssertionError

    def compute_friction_cone(self, phi_B):
        """
        Compute the bases of the friction cone.
        @param phi_B: The yaw of the contact point in the B frame.
        @return: (p_W, b0_W, b1_W). p_W is the 3D position of the apex.
        b0_W and b1_W are 3D unit basis vectors spanning the friction cone.
        By convention, the yaw increases from b0_W to b1_W.
        Since we are in 2D, the z's of all 3 vectors should be identical.
        """
        p_W, n_W = self.compute_p_W_and_n_W(phi_B)
        cos = np.cos(self.half_friction_cone_aperture)
        sin = np.sin(self.half_friction_cone_aperture)
        assert(cos >= 0)
        assert(sin >= 0)
        R = np.zeros((3, 3))
        R[:2, :2] = np.asarray([[cos, -sin], [sin, cos]])
        b0_W = np.dot(R.T, n_W)
        b1_W = np.dot(R, n_W)
        assert(b0_W[-1] == 0)
        assert (b1_W[-1] == 0)
        return p_W, b0_W, b1_W

    def compute_f_W_p(self, phi_B, alpha_0, alpha_1):
        """
        Compute the force applied on the object as ᵂfᵖ=α₀⋅ᵂb₀+α₁⋅ᵂb₁
        @param phi_B: The yaw of the contact point in the B frame.
        @param alpha_0: coefficient on b0_W
        @param alpha_1: coefficient on b1_W
        @return: (p_W, f_W_p), p_W is the 3D contact point on the object.
        f_W_p is the force applied
        """
        assert alpha_0 >= 0
        assert alpha_1 >= 0
        # First compute the force vectors
        p_W, b0_W, b1_W = self.compute_friction_cone(phi_B)
        # Compute the force
        f_W_p = alpha_0*b0_W+alpha_1*b1_W
        assert f_W_p[-1] == 0
        return p_W, f_W_p


class DefaultBoxObject(Object):
    def __init__(self, p_W_init=None, q_W_init=None, **kwargs):
        super().__init__(os.path.dirname(
            os.path.realpath(__file__))+"/assets/2x1x05.sdf", p_W_init,
            q_W_init, **kwargs)


class BoxVisualization(Object):
    def __init__(self, p_W_init=None, q_W_init=None, **kwargs):
        super().__init__(os.path.dirname(
            os.path.realpath(__file__))+"/assets/2x1x05_vis.sdf", p_W_init,
            q_W_init, **kwargs)
        # Change color to green
        p.changeVisualShape(1, -1, rgbaColor=[0., 0.8, 0., 0.4])

