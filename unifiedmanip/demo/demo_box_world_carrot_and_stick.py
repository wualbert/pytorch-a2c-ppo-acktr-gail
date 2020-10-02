import sim.world as world
import sim.agent as agent
import pybullet as p
import time

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
box_world = world.BoxWorld()
agnt = agent.CarrotAndStickAgent(list(box_world.object_dict.values())[0])
box_world.agent_list.append(agnt)

for i in range(10000):
    box_world.step_simulation()
    time.sleep(1./240.)
p.disconnect()
