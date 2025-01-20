from navground import sim, core
import numpy as np

class HomeScenario(sim.Scenario, name='Home'):
    def init_world(self, world, seed):
        super().init_world(world, seed=seed)
        world.set_lattice(1, (0, 13))
         
        world.add_wall(sim.Wall((0,2),(6,2)))
        world.add_wall(sim.Wall((8.5,2),(14,2)))
        world.add_wall(sim.Wall((0,11),(6,11)))
        world.add_wall(sim.Wall((8.5,11),(14,11)))
        world.add_wall(sim.Wall((0,2),(0,11)))
        world.add_wall(sim.Wall((14,2),(14,11)))
        world.add_wall(sim.Wall((0,5),(6,5)))
        world.add_wall(sim.Wall((0,8),(6,8)))
        world.add_wall(sim.Wall((6,1.25),(6,2.75)))
        world.add_wall(sim.Wall((6,4.25),(6,5.75)))
        world.add_wall(sim.Wall((6,7.25),(6,8.75)))
        world.add_wall(sim.Wall((6,10.25),(6,11.75)))
        world.add_wall(sim.Wall((8.5,1.25),(8.5,5.75)))
        world.add_wall(sim.Wall((8.5,7.25),(8.5,11.75)))

        p00 = (1.5,3.5)
        p01 = (1.5,6.5)
        p02 = (1.5,9.5)
        p10 = (7.25,3.5)
        p11 = (7.25,6.5)
        p12 = (7.25,9.5)
        p20 = (12.5,3.5)
        p21 = (12.5,6.5)
        p22 = (12.5,9.5)
        p0 = (7.25,0.5)
        p1 = (7.25,12.5)
        p2 = (9.25,6.5)
                
        if len(world.agents) > 0:
            world.agents[0].position = (np.random.uniform(1, 5),3.5)
            world.agents[0].color = 'gold'
        if len(world.agents) > 1:
            world.agents[1].position = (np.random.uniform(1, 5),6.5)
            world.agents[1].color = 'orange'
        if len(world.agents) > 2:
            world.agents[2].position = (np.random.uniform(1, 5),9.5)
            world.agents[2].color = 'orangered'
        if len(world.agents) > 3:
            h = np.random.uniform(3,11)
            world.agents[3].position = (7.95, h)
            world.agents[3].orientation = np.pi/2
            world.agents[3].color = 'greenyellow'
        if len(world.agents) > 4:
            world.agents[4].position = (6.65, h)
            world.agents[4].orientation = np.pi/2
            world.agents[4].color = 'cyan'
     
        for i, agent in enumerate(world.agents):
            if agent.task is None:
                if i % 5 == 0:
                    agent.task = sim.tasks.WaypointsTask(waypoints=[p00,p10,p11,p2,p22,p2,p11,p10], loop=True, tolerance=1)
                elif i % 5 == 1:
                    agent.task = sim.tasks.WaypointsTask(waypoints=[p01,p21], loop=True, tolerance=1)
                elif i % 5 == 2:
                    agent.task = sim.tasks.WaypointsTask(waypoints=[p02,p12,p11,p2,p20,p2,p11,p12], loop=True, tolerance=1)
                elif i % 5 == 3:
                    agent.task = sim.tasks.WaypointsTask(waypoints=[p0,p10,p11,p12,p1,p12,p11,p10,p0], loop=True, tolerance=1)
                else:
                    agent.task = sim.tasks.WaypointsTask(waypoints=[p0,p10,p11,p12,p1,p12,p11,p10,p0], loop=True, tolerance=1)