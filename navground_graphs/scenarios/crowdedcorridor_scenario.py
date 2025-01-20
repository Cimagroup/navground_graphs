from navground import sim, core
import numpy as np

class CrowdedCorridorScenario(sim.Scenario, name='CrowdedCorridor'):
    def init_world(self, world, seed):
        super().init_world(world, seed=seed)
        world.set_lattice(0, (0, 30))
         
        world.bounding_box = sim.BoundingBox(0, 30, 0, 3)
        world.add_wall(sim.Wall((0,0),(30,0)))
        world.add_wall(sim.Wall((0,3),(30,3)))
        
        if len(world.agents) > 0:
            world.agents[0].position = (5, 0.55)
            world.agents[0].orientation = 0
        if len(world.agents) > 1:
            world.agents[1].position = (25, 1)
            world.agents[1].orientation = np.pi
        if len(world.agents) > 2:
            world.agents[2].position = (5, 1.50)
            world.agents[2].orientation = 0
        if len(world.agents) > 3:
            world.agents[3].position = (25, 2)
            world.agents[3].orientation = np.pi
        if len(world.agents) > 4:
            world.agents[4].position = (5, 2.45)
            world.agents[4].orientation = 0

        direction = 1
        for agent in world.agents:
            if agent.task is None:
                agent.task = sim.tasks.DirectionTask(direction=(direction, 0))
                direction *= -1