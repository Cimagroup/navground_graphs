from navground import sim, core
import numpy as np

class BowtieScenario(sim.Scenario, name='Bowtie'):
    def init_world(self, world, seed):
        super().init_world(world, seed=seed)
        world.set_lattice(0, (0, 30))
         
        world.bounding_box = sim.BoundingBox(0, 30, 0, 3)
        world.add_wall(sim.Wall((0,0),(30,0)))
        world.add_wall(sim.Wall((0,3),(30,3)))
        world.add_wall(sim.Wall((13,0),(15,0.75)))
        world.add_wall(sim.Wall((13,3),(15,2.25)))
        world.add_wall(sim.Wall((15,0.75),(17,0)))
        world.add_wall(sim.Wall((15,2.25),(17,3)))
        
        if len(world.agents) > 0:
            world.agents[0].position = (2, 0.8)
            world.agents[0].orientation = 0
        if len(world.agents) > 1:
            world.agents[1].position = (28, 0.8)
            world.agents[1].orientation = np.pi
        if len(world.agents) > 2:
            world.agents[2].position = (2, 2.2)
            world.agents[2].orientation = 0
        if len(world.agents) > 3:
            world.agents[3].position = (28, 2.2)
            world.agents[3].orientation = np.pi

        direction = 1 
        for agent in world.agents:
            if agent.task is None:
                agent.task = sim.tasks.DirectionTask(direction=(direction, 0))
                direction *= -1