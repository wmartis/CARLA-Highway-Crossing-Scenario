import time
import carla
import logging
import py_trees
import numpy as np
import threading

from srunner.scenariomanager.timer import TimeOut, GameTime
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AtomicBehavior
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerRegion



actor_list = []

class DestroyActors(AtomicBehavior):
    def __init__(self):
        super(DestroyActors, self).__init__("DestroyActors")
        
    def update(self):
        client = CarlaDataProvider.get_client()
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])

        return py_trees.common.Status.SUCCESS

class AccelerateSimulation(AtomicBehavior):
    def __init__(self):
        super(AccelerateSimulation, self).__init__("AccelerateSimulation")

    def update(self):
        world = CarlaDataProvider.get_world()
        settings = world.get_settings()
        settings.fixed_delta_seconds = 0.05
        
        world.apply_settings(settings)

        return py_trees.common.Status.SUCCESS


class RestoreSimulation(AtomicBehavior):
    def __init__(self):
        super(RestoreSimulation, self).__init__("RestoreSimulation")

    def update(self):
        world = CarlaDataProvider.get_world()
        settings = world.get_settings()
        settings.fixed_delta_seconds = 0
       
        world.apply_settings(settings)

        return py_trees.common.Status.RUNNING

class HighwayCross(BasicScenario):

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True):
    
        super(HighwayCross, self).__init__("TestScenario",
                                                ego_vehicles,
                                                config,
                                                world,
                                                debug_mode,
                                                criteria_enable=criteria_enable)

        

    def _initialize_actors(self, config):
        world = CarlaDataProvider.get_world()
        client = CarlaDataProvider.get_client()
        tm = client.get_trafficmanager(8000)
        tm.global_percentage_speed_difference(-1000)

        settings = world.get_settings()
        
        self.stopThread = False
        self.trafficThread = threading.Thread(target=self.generate_traffic, daemon=True)
        self.trafficThread.start()
        #trafficThread.join()
       
        settings.fixed_delta_seconds = 0.1
        
        world.apply_settings(settings)
        time.sleep(0.4)
        settings.fixed_delta_seconds = 0
        world.apply_settings(settings)

    def generate_traffic(self):

        world = CarlaDataProvider.get_world()
        client = CarlaDataProvider.get_client()
        blueprint_library = world.get_blueprint_library()
        tm = client.get_trafficmanager(8000)
        spawn_points = [carla.Transform(carla.Location(x=340, y=38.5+i*3.35, z=0.5), carla.Rotation()) for i in range(5)]
    
        vehicles = blueprint_library.filter('vehicle.*')
        vehicles = [i for i in vehicles if int(i.get_attribute('number_of_wheels')) != 2 ]

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor
        synchronous_master = False

        

        #np.random.seed(7)

        
        for x in range(24):
            
            batch = []
            spawns = np.random.binomial(1, 0.5, 5)
            for i, point in enumerate(spawn_points):
                if spawns[i]:
                    vehicle_bp = np.random.choice(vehicles)
                    batch.append(SpawnActor(vehicle_bp, point)
                    .then(SetAutopilot(FutureActor, True, tm.get_port()))
                    )

            for response in client.apply_batch_sync(batch, synchronous_master):
                if response.error:
                    logging.error(response.error)
                else:
                    actor_list.append(response.actor_id)

            stop = world.get_snapshot().timestamp.elapsed_seconds + 5
            while world.get_snapshot().timestamp.elapsed_seconds < stop:
                pass
            
            
            #if x < 5:
                #time.sleep(0.1)
            #else:
                #time.sleep(5)
            
            

    def _create_behavior(self):
  
        trafficSequence = py_trees.composites.Sequence("Sequence Traffic")
        for i in range(5):
            trafficSequence.add_child(TrafficBehavior())
            trafficSequence.add_child(TimeOut(5))
        trafficSequence.add_child(TimeOut(20))

        skipSequence = py_trees.composites.Sequence("Sequence Skip")
        skipSequence.add_child(AccelerateSimulation())
        skipSequence.add_child(TimeOut(10))
        skipSequence.add_child(RestoreSimulation())

        endcondition = py_trees.composites.Parallel("Waiting for end position", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        trigger = InTriggerRegion(self.ego_vehicles[0], 500, 520, 0, 30)


        #endcondition.add_child(trafficSequence)
        endcondition.add_child(trigger)
        #endcondition.add_child(skipSequence)

        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(endcondition)
        sequence.add_child(DestroyActors())
        

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        #self.stopThread = True
        self.remove_all_actors()
        import sys
        sys.exit()


class TrafficBehavior(AtomicBehavior):
    def __init__(self):
        super(TrafficBehavior, self).__init__("TrafficBehavior")

    def update(self):
        world = CarlaDataProvider.get_world()
        client = CarlaDataProvider.get_client()
        blueprint_library = world.get_blueprint_library()
        tm = client.get_trafficmanager(8000)

        self.spawn_points = [carla.Transform(carla.Location(x=340, y=38.5+i*3.35, z=0.5), carla.Rotation()) for i in range(5)]
    
        vehicles = blueprint_library.filter('vehicle.*')
        self.vehicles = [i for i in vehicles if int(i.get_attribute('number_of_wheels')) != 2 ]

        SpawnActor = carla.command.SpawnActor
        SetAutopilot = carla.command.SetAutopilot
        FutureActor = carla.command.FutureActor
        synchronous_master = False

        spawns = np.random.binomial(1, 0.5, 5)

        #np.random.seed(7)

        batch = []
        for i, point in enumerate(self.spawn_points):
            if spawns[i]:
                vehicle_bp = np.random.choice(self.vehicles)
                batch.append(SpawnActor(vehicle_bp, point)
                .then(SetAutopilot(FutureActor, True, tm.get_port()))
                )

        for response in client.apply_batch_sync(batch, synchronous_master):
            if response.error:
                logging.error(response.error)
            else:
                actor_list.append(response.actor_id)

        return py_trees.common.Status.SUCCESS 
