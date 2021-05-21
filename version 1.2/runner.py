#!/usr/bin/env python


from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
from collections import defaultdict
# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import AutoVehicle
import SingleAgent
import env
from sumolib import checkBinary  # noqa
import traci  # noqa

def generate_routefile():
    random.seed(420)  # make tests reproducible
    N = 360  # number of time steps

    with open("data/cross.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="car" accel="3.5" decel="3.5" sigma="0.5" width="1.8" length="4.5" minGap="0" maxSpeed="25.2" guiShape="passenger"/>

        <route id="right" edges="51o 1i 2o 52i" />
        <route id="right_down" edges="51o 1i 3o 53i" />
        <route id="right_up" edges="51o 1i 4o 54i" />
        <route id="left" edges="52o 2i 1o 51i" />
        <route id="left_down" edges="52o 2i 3o 53i" />
        <route id="left_up" edges="52o 2i 4o 54i" />

        <route id="up" edges="53o 3i 4o 54i" />
,        <route id="up_right" edges="53o 3i 2o 52i" />
        <route id="up_left" edges="53o 3i 1o 51i" />
        <route id="down_right" edges="54o 4i 2o 52i" />
        <route id="down_left" edges="54o 4i 1o 51i" />
        <route id="down" edges="54o 4i 3o 53i" />
        """, file=routes)
        vehNr = 0
        # demand per second from different directions
        p = 1/100

        myroutes = ['right','right_up','right_down','left','left_up','left_down',"up", "up_right" , "up_left" , "down_right" , "down_left" , "down"]
        for i in range(N):
            for route in myroutes:
                r = random.uniform(0,1)
                if r < p:
                    print(f'        <vehicle id="{route}_{vehNr}" type="car" route="{route}" depart="{i}" />' , file=routes)
                    vehNr += 1
        print("</routes>", file=routes)

def run():
    """execute the TraCI control loop"""
    step = 0
    #vechilesDict = generate_routefile()
    generate_routefile()
    #print(vechilesDict)
    existing_agents = []
    # traci.vehicle.setColor()
    # # car generation
    # id_list = traci.vehicle.getIDList()
    # vehicleList =[]
    # vechile_dict ={}
    # print(len(id_list))
    # for id in id_list:
    #     print(id)
    #     car = AutoVehicle.AutoVehicle(id)
    #     vehicleList.append(car)
    #     vechile_dict[id] = car
    # car2 = AutoVehicle.AutoVehicle("LH")
    # car3 = AutoVehicle.AutoVehicle("D2")
    # car4 = AutoVehicle.AutoVehicle("LH")
    # car = AutoVehicle.AutoVehicle("agent")
    # #vehicleList = [car , car2 , car3 , car4]
    #
    my_env = env.env()
    # agent = SingleAgent.SingleAgent(my_env, car)
    # # agent2 = SingleAgent.SingleAgent( my_env , car2)
    # car_dict = {"agent":car }
    # my_env.vehicleList = [SingleAgent.SingleAgent(my_env, car) for car in list(car_dict.values())]
    #
    while traci.simulation.getMinExpectedNumber() > 0:
        step += 1
        time = traci.simulation.getTime()
        traci.simulationStep()
        arv = traci.simulation.getArrivedIDList()
        dep = traci.simulation.getDepartedIDList()

        # ADD newly added cars as an agents
        for car_id in dep:
            existing_agents.append(SingleAgent.SingleAgent(my_env, AutoVehicle.AutoVehicle(car_id)))

        # Remove the arrived-to-destination cars
        for agent in existing_agents:
            if agent.car.ID in arv:
                existing_agents.remove(agent)

        # Cars in
        my_env.updateIntersectionAgents(existing_agents)
        my_env.updateStates()

        for agent in my_env.intersectionAgentList:
            joint = my_env.is_overlap(agent)
            if len(joint) > 0:
                print(time, joint)

        # for x in my_env.intersectionAgentList:
        #     print(x.car.ID)
        #Creating a list of cars in the intersection:



      # #  [print(x.car.ID) for x in existing_cars]

    # Get the newly added cars
        # try: new_cars = vechilesDict[time]
        # except: new_cars = []
        #
        # # ADD the new cars to agent list
        # for car_id in new_cars:
        #     existing_cars.append(SingleAgent.SingleAgent(my_env, AutoVehicle.AutoVehicle(car_id)))
        #
        # dep = list(traci.simulation.getDepartedIDList()) # Get the cars getting out of the simulation
        # print(time)
        # # for x in existing_cars:
        # #     print(x.car.ID)
        # print(dep)
        #     # Remove the departed cars from the agent list
        # for i, agent in enumerate(existing_cars):
        #     if agent.car.ID in dep:
        #         print('removing:', agent.car.ID)
        #         existing_cars.remove(existing_cars[i])

        # if existing_cars != [] :print(existing_cars)

        # if True:
        #     # print(time)
        #     [print(x.car.ID) for x in existing_cars]
        #     # [print(x) for x in dep]

        # if dep != (): print(dep)
        # print(traci.simulation.getTime())
        # print(traci.vehicle.getIDList())
    #     #
    #     # id_list = traci.vehicle.getIDList()
    #     # vehicleList = [] # in car generation
    #     # vechile_dict = {}  # in car generation
    #     # print(len(id_list))
    #     id_list = traci.vehicle.getIDList()
    #     car.UpdateStatus()
    #     for id in id_list:
    #         car = car_dict[id]
    #         print(traci.vehicle.getLaneID(car.ID))
    #         car.UpdateStatus()
    #         # print(car.pos)
    #         agent = SingleAgent.SingleAgent(my_env, car)
    #         if car.inIntersection():
    #             curr_cells, desired_cells = my_env.get_current_cells(agent)
    #             agent.PickAction()
    #             agent.TakeAction()
    #             print(car.queue)
    #
    #             # print(curr_cells, desired_cells, car.currentspeed, traci.vehicle.getSpeed(car.ID))
    #             # print(car.currentspeed, car)
    #
    #         # print(car.inIntersection())
    #     #  x .1 x'
    #     #  x .5 x'
    #     step += 1
    #     # my_env.vehicleList = []
    #     #id_list = traci.vehicle.getIDList()
    #
    #     # Intersection Testing (Passed)
        # for vehicle in vehicleList:
        #     try:
        #         if vehicle.inIntersection():
        #             my_env.vehicleList.append(vehicle)
        #     except:
        #         pass


        # if step%7 == 0:
        #     print(step)
        #     agent.changeLane("left" , step)
        #     print("Hey , i changed my lane to left ")
        # if step % 33 == 0:
        #     print(step)
        #     agent.changeLane("right"  , step)
        #     print("Hey , i changed my lane to right")

        # myenv = AutoVehicle.env(agent)
        # myenv.PickAction()
        # myenv.TakeAction( step )
        # myenv.Current_state()
        #try:
        #current_state =  my_env.Car_current_state(agent)
        #possible_actions = my_env.getFeasibleActions(agent)
        #agent.PickAction(current_state , possible_actions)
        #agent.TakeAction(step)
        #my_env.Reward(agent)

        # my_env.
        # agent1.PickAction()
        # agent1.TakeAction(step)
        # my_env.Current_state(agent1)
        # my_env.Reward(agent1)
        #except:
         #   pass
        #car2.getPose()
        # car.getBoxintersection()
        #car2.getBoxintersection()
        #car3.getBoxintersection()
        #car.getBoxintersection()
        # car.printPose()
        # car.printPose(500,4,20)

#[:0_9 , :0_1 , :0_5 ,:0_13]




    traci.close()
    sys.stdout.flush()


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    optParser.add_option("--start", action="store",
                          default=0, help="skip to specific time")
    optParser.add_option("--step", action="store",help="set the step length to simulate at")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    print(sumoBinary)
    # first, generate the route file for this simulation
    #generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    if options.step is None:
        options.step = .1
    print((options.step))
    traci.start([sumoBinary, "-c", "data/cross.sumocfg",
                             "--tripinfo-output", "tripinfo.xml",
                             "--step-length", ".5"]
                             # "--begin", "400"]
    )
    run()
