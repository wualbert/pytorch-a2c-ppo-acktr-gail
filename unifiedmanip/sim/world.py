import sim.rigid_object_deprecated as object

import pybullet as p
import pybullet_data


class World:
    """
    The World is the bookkeeping entity for a simulation. It contains objects
    (currently 1) and agents. The objects are defined in rigid_object_deprecated.py.
    The World helps sets up the pybullet environment and keeps track of stuff
    when running simulation.
    """

    def __init__(self, object_list=None, agent_list=None,
                 sim_time_step=1./240.,
                 gravity=(0., 0., -10.), load_floor=True,
                 floor_friction=0., **kwargs):
        if object_list is None:
            object_list = []
        if agent_list is None:
            agent_list = []
        if sim_time_step != 1./240.:
            p.setTimeStep(sim_time_step)
        self.sim_time_step = sim_time_step
        self.object_dict = dict()
        for obj in object_list:
            self.object_dict[obj.body_id] = obj
        self.agent_list = agent_list
        self.gravity = gravity
        p.setGravity(*self.gravity)
        self.floor_friction = floor_friction
        if load_floor:
            p.setAdditionalSearchPath(
                pybullet_data.getDataPath())  # optionally
            self.floor_id = p.loadURDF("plane.urdf")
            p.changeDynamics(self.floor_id, -1,
                             lateralFriction=self.floor_friction)
        # Take one simulation step
        p.stepSimulation()

    def step_simulation(self):
        # Get the actions from all agents
        # TODO(wualbert)
        # Execute the actions
        # TODO(wualbert)
        # Take one simulation step
        p.stepSimulation()


class BoxWorld(World):
    """
    World with one 2x1x0.5 rectangular box
    """

    def __init__(self, p_W_init=(0., 0., 0.25), q_W_init=(0., 0., 0., 1.),
                 agent_list=None, sim_time_step=1./240.,
                 gravity=(0., 0., -10.),
                 target_p_W = (3.,0.,0.25), target_q_W = (0.,0.,0.,1.)):

        objects_list = [object.DefaultBoxObject(p_W_init, q_W_init)]
        if target_p_W is not None and target_q_W is not None:
            objects_list.append(object.BoxVisualization(target_p_W, target_q_W))
        super().__init__(objects_list, agent_list, sim_time_step, gravity)

    def step_simulation(self):
        # The world currently relies on applyExternalForce for actions
        for agnt in self.agent_list:
            agnt.perform_action()
        p.stepSimulation()
