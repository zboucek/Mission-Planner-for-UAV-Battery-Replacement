#!/usr/bin/env python3

import asyncio

from mavsdk import System
import cloudpickle


async def run(drone_address = "udp://:14540", mission_plan_name = "mission_plan"):

    drone = System()
    await drone.connect(system_address=drone_address)
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {drone._sysid}")
            break
    
    mission_plan = await drone.mission_raw.download_mission()
    print("-- Mission plan downloaded succesfully")
    print(mission_plan)
    if len(mission_plan) > 0:
        cloudpickle.dump(mission_plan, open(mission_plan_name + ".pkl", "wb"))
    
if __name__ == "__main__":
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run("udp://:14540", "missions/mission_plan_X"))