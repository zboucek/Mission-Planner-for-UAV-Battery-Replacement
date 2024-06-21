import numpy as np


class Battery(object):
    """Battery representation in DronePort system.
    It can be placed in DronePort charging/storage station or drone.

    Input:
        id: Battery id number
        max_cap: maximum capacity of battery
        current_cap: current capacity of battery
    """

    def __init__(self, id=-1, max_cap=1.0, current_cap=1.0):
        self.id = id
        self.max_capacity = max_cap  # maximum capacity of battery
        # Set current capacity.
        if current_cap > self.max_capacity:
            self.current_capacity = self.max_capacity
        else:
            self.current_capacity = current_cap

    def charge(self):
        """Change the battery."""
        self.current_capacity = self.max_capacity
        self.print(f"Battery #{self.id} charged!")

    def run(self, discharge=-0.01):
        """Discharge the battery."""
        if discharge < 0:
            self.current_capacity = self.current_capacity + discharge
            if self.current_capacity < 0:
                self.current_capacity = 0
                self.print(f"Battery #{self.id} is empty!")
        else:
            self.print(f"Wrong value for discharge: {discharge}")


class STATE:
    unknown = 0
    start = 1
    connected = 2
    upload = 3
    mission = 4
    backup = 5
    upload_dp = 6
    mission_dp = 7
    land_dp = 8
    swap = 9
    reupload = 10
    error = 15

    # desc = {
    #     unknown: "unknown",
    #     start: "waiting for connection",
    #     connected: "heartbeat acknowledged",
    #     upload: "receiving new mission",
    #     mission: "mission in progress",
    #     backup: "backing up mission and saving to file",
    #     upload_dp: "receiving mission to DronePort",
    #     mission_dp: "on mission to DronePort",
    #     land_dp: "landing on DronePort",
    #     swap: "DronePort is swapping the battery",
    #     reupload: "receiving original mission",
    #     error: "some error occured"
    # }

    desc = {
        0: "unknown",
        1: "waiting for HB",
        2: "Clearing mission",
        3: "uploading mission",
        4: "mission started",
        5: "mission in progress",
        6: "RTL to DronePort",
        7: "wait for land DronePort",
        8: "wait for battery",
        15: "some error occured"
    }


class Drone(object):
    """Drone uses pyMAVLink library for communication.
    Also, it contains Battery object and its current mission.
    """

    def __init__(self, battery=Battery(),
                 source_system=254, source_component=1, target_system=1, target_component=1,
                 max_speed=5.0, max_flight_time=15*60, location=[.0, .0, .0]):
        """Establish connection with drone using MAVLink.

        Input:
            address: string with IP address
            port: string with PORT
            battery: object of Battery class
            sys_id: id of Controlling system
            id: id of component
        """
        self.id = target_system
        self.armed = False
        # State of the drone state machine
        self._state = STATE.unknown

        # Init mission variables
        self.mission_items = []
        self.time_plan = []
        self.resume_item = -1
        self.mission_current = -1
        self.mission_count = -1
        self.mission_soc = []
        self.wp = []
        self.out_file = f"park_drone{self.id}_short_out.pkl"
        self.in_file = f"park_drone{self.id}_short.pkl"
        self.max_speed = max_speed  # max speed in m/s
        self.max_flight_time = max_flight_time  # max flight time in s

        # Drone status
        self.battery = battery
        self.location = location
        self.landed_state = -1  # -1 unknown, 1 on ground, 2 in air, 3 takeoff, 4 landing
        self.verbose = False

        self.lock = None

    def print(self, msg):
        if self.verbose:
            print(f'[Drone{self.id}] {msg}')
        self.status_msg = msg

    def fly2point(self, coordinates=[0, 0, 0], discharge=0.1):
        self.location = coordinates
        self.battery.run(discharge)
        print(f'new coordinates: {self.location}')

    def reset_battery(self):
        """Reset simulation of battery in drone."""
        self.print('Reset battery requested')

        self.battery.charge()

    def new_mission(self, mission_items, time_plan = None, current=0):
        """
        Assign a new mission to the drone and update additional variables accordingly.

        For correct evaluation, the initial position of drone is included into the mission list.
        """
        self.mission_items = np.vstack((self.location, mission_items))
        
        if time_plan is None:
            # time_plan = np.zeros((mission_items.shape[0], 1))
            distances = np.zeros((mission_items.shape[0], 1))
            # distance from mission waypoint to another
            for i in range(mission_items.shape[0]-1):
                distances[i,0] = self.distance(mission_items[i], mission_items[i+1])
            time_plan = np.cumsum(self.distance2time_arr(distances))
        
        time2start = self.distance2time_arr(
            self.distance(self.location, mission_items[0,:]))
        self.time_plan = np.vstack(([time2start], time_plan.reshape((-1, 1)) + time2start))
        self.mission_current = current
        self.mission_count = self.mission_items.shape[0]
        self.mission_soc = self.time2soc()

    def gen_circ_mission(self, t_max=10*60, samples=100, x=0, y=0, r_x=10, r_y=10):
        """
        Return mission and time vector with random points given on ellipse.
        Ellipse is given by its center x,y and radius r_x, r_y.
        Time vector is randomly generated with given samples and t_max as max value.
        """
        time = np.sort(t_max*np.random.random((samples, 1)), axis = 0)
        mission = np.hstack((r_x*np.cos(time)+x, r_y*np.sin(time)+y))
        self.new_mission(mission, time)

        return mission, time

    def gen_circ_rand_mission(self, samples=100, min_r=10, var_r=10):
        """
        Return mission and time vector with random points random on ellipse.
        Ellipse is given by the location of drone in center x,y and random radius generated on interval [min_r, min_r+var_r].
        Time vector is randomly generated with given samples and drone max flight time as max value.
        """
        mission, time = self.gen_circ_mission(t_max=2.0*self.max_flight_time, samples=samples, x=self.location[0, 0],
                                              y=self.location[0, 1], r_x=var_r*np.random.random()+min_r, r_y=var_r*np.random.random()+min_r)
        return mission, time

    def distance2time_arr(self, distance):
        """
        Convert distance [m] array arr to time [s] array according to speed [m/s]
        and return a time vector.
        """
        return distance/self.max_speed

    def time2soc(self):
        """
        Returns State-of-Charge (SoC) [0-1] decrease based on mission flight time intervals 
         and maximum flight time [s] (when SoC is 1.0).
        """
        return np.hstack((0, np.diff(self.time_plan[:, 0]/self.max_flight_time)))

    def distance(self, a, b):
        """
        Return euclidian distance between point a and b.
        """
        return np.linalg.norm(b-a, 2)

    def distances2point(self, point):
        """
        Return euclidian distance between waipoints array and point.

        waypoints: points are in rows of array
        """

        d = np.zeros((self.mission_items.shape[0], 1))

        for i, elem in enumerate(self.mission_items):
            d[i] = self.distance(elem, point)

        return d

    def soc_and_time2point(self, point):
        """
        Return State-of-Charge (SoC) [0-1] decrease array caused by travel from waypoint to point.
         """
        d = self.distances2point(point)
        t = self.distance2time_arr(d)
        soc = t/self.max_flight_time

        return soc, t


class Droneport(object):
    """DronePort landing platform with smart charging station and robotic manipulator.

    Input:
        drone: link to Drone object which occupies the DronePort
        batteries: present batteries
        sys_id: id of Controlling system
        id: id of DronePort
        location: array with latitude, longitude, altitude in degrees and meters
        slots: number of slots
    """

    def __init__(self, drone=None, batteries=[],
                 source_system=254, source_component=1, target_system=201, target_component=1, location=[.0, .0, .0],
                 slots=3, swap_time = 60):

        self.sys_id = source_system
        self.comp_id = source_component
        self.id = target_system
        # geo coordinates: latitude, longitude, altitude in degrees and meters
        self.location = location
        self.drone = drone    # boolean value: Drone object occupying DronePort landing platform
        self.batteries = batteries  # array with Battery objects
        self.slots = slots  # number of DronePort battery slots
        self.swap_status = 0
        self.schedule = []  # time schedule for battery swapping
        self.swap_time = swap_time  # time needed for swapping the drone's battery
