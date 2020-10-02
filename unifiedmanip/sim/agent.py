import pybullet as p


class Agent:
    def __init__(self, **kwargs):
        pass

    def perform_action(self, **kwargs):
        pass


class CarrotAndStickAgent(Agent):
    """
    Push the object in the front at the edge of the friction cone
    """

    def __init__(self, obj, phi=0., alpha_0=20., alpha_1=0.):
        super().__init__()
        self.obj = obj
        self.phi = phi
        self.alpha_0 = alpha_0
        self.alpha_1 = alpha_1

    def perform_action(self, **kwargs):
        p_W, f_W_p = self.obj.compute_f_W_p(
            self.phi, self.alpha_0, self.alpha_1)
        p.applyExternalForce(self.obj.body_id, -1, f_W_p, p_W, p.WORLD_FRAME)
