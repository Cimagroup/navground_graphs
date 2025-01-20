from navground import sim, core
import numpy as np

class MaltaCrossScenario(sim.Scenario, name='MaltaCross'):
    def init_world(self, world, seed):
        super().init_world(world, seed=seed)
        world.set_lattice(0, (0, 21))
        world.set_lattice(1, (0, 21))
         
        world.add_wall(sim.Wall((0,5),(9.75,9.75)))
        world.add_wall(sim.Wall((11.25,9.75),(21,5)))
        world.add_wall(sim.Wall((0,16),(9.75,11.25)))
        world.add_wall(sim.Wall((11.25,11.25),(21,16)))
        world.add_wall(sim.Wall((5,0),(9.75,9.75)))
        world.add_wall(sim.Wall((9.75,11.25),(5,21)))
        world.add_wall(sim.Wall((16,0),(11.25,9.75)))
        world.add_wall(sim.Wall((11.25,11.25),(16,21)))
        
        if len(world.agents) > 0:
            world.agents[0].position = (2.05, 10.5)
            world.agents[0].orientation = 0
        if len(world.agents) > 1:
            world.agents[1].position = (10.5, 1)
            world.agents[1].orientation = np.pi/2
        if len(world.agents) > 2:
            world.agents[2].position = (19.05, 10.5)
            world.agents[2].orientation = np.pi
        if len(world.agents) > 3:
            world.agents[3].position = (10.5, 19)
            world.agents[3].orientation = -np.pi/2
     
        for i, agent in enumerate(world.agents):
            if agent.task is None:
                if i % 4 == 0:
                    agent.task = sim.tasks.DirectionTask(direction=(1,0))
                elif i % 4 == 1:
                    agent.task = sim.tasks.DirectionTask(direction=(0,1))
                elif i % 4 == 2:
                    agent.task = sim.tasks.DirectionTask(direction=(-1,0))
                else:
                    agent.task = sim.tasks.DirectionTask(direction=(0,-1))