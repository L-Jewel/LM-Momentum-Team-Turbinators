#!/usr/bin/env python3

import asyncio
import pygazebo
import pygazebo.msg.v11.laserscan_stamped_pb2 # Imports LiDAR readouts
import pygazebo.msg.v11.gps_pb2 # Imports GPS readouts
import math
import time
import numpy as np

from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

# The gazebo master from PX4 message
HOST, PORT = "127.0.0.1", 11345

class GazeboMessageSubscriber:
    def __init__(self, host, port, timeout=30): # Initializes the class
        self.host = host
        self.port = port
        self.loop = asyncio.get_event_loop()
        self.running = False
        self.timeout = timeout

    async def connect(self):
        connected = False

        # I believe that this attempts to connect to the sensors for [self.timeout] seconds (default 30)
        for i in range(self.timeout):
            try:
                self.manager = await pygazebo.connect((self.host, self.port))
                connected = True
                break
            except Exception as e:
                print(e)
            await asyncio.sleep(1)

        # If a successful connection is made, the script enters this if statement here
        if connected:
            # These variables represent the topic and message for both of the sensors
            lidar_topic = '/gazebo/default/iris_lmlidar/lmlidar/link/lmlidar/scan'
            lidar_msg = 'gazebo.msgs.LaserScanStamped'
            gps_topic = '/gazebo/default/iris_lmlidar/gps0/link/gps'
            gps_msg = 'gazebo.msgs.GPS'

            # These next few lines await the sensor data
            self.lidar_subscriber = self.manager.subscribe(lidar_topic, lidar_msg, self.LaserScanStamped_callback)
            self.gps_subscriber = self.manager.subscribe(gps_topic, gps_msg, self.gps_callback)

            await self.lidar_subscriber.wait_for_connection()
            await self.gps_subscriber.wait_for_connection()

            self.running = True
            while self.running:
                await asyncio.sleep(0.1)
        else:
            raise Exception("Timeout connecting to Gazebo.")

    def LaserScanStamped_callback(self, data):
        self.LaserScanStamped = pygazebo.msg.v11.laserscan_stamped_pb2.LaserScanStamped()
        self.LaserScanStamped.ParseFromString(data)

    def gps_callback(self, data):
        self.GPS = pygazebo.msg.v11.gps_pb2.GPS()
        self.GPS.ParseFromString(data)

    async def get_LaserScanStamped(self):
        for i in range(self.timeout):
            try:
                return self.LaserScanStamped
            except Exception as e:
                # print(e) <-- For debugging, if we want to see the error message
                pass
            await asyncio.sleep(1)

    async def get_GPS(self):
        for i in range(self.timeout):
            try:
                return self.GPS
            except Exception as e:
                # print(e) <-- For debugging, if we want to see the error message
                pass
            await asyncio.sleep(1)

# Creates an instance of GazeboMessageSubscriber to store sensor data
gz_sub = GazeboMessageSubscriber(HOST, PORT)
asyncio.ensure_future(gz_sub.connect())

async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    print("Observing in air")

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return

'''
Converts the data from the LiDAR sensor into a usable data structure
Argument: An instance of GazeboMessageSubscriber
Returns a dictionary with the following values:
sec: current time in seconds
nsec: current time in nanoseconds(?)
x_pos: x coord of drone
y_pos: y coord of drone
z_pos: z coord of drone

x_ori: roll?
y_ori: pitch?
z_ori: yaw?
w_ori: no clue what this is
^ Tbh, I don't know what these mean. They're just under the orientation bracket
h_angle_max: Max horizontal angle (pi/6)
h_angle_min: Min horizontal angle (-pi/6)
h_angle_step: Step by which each horizontal angle increases
h_angle_count: Number of horizontal scan lines (20)
v_angle_max: Max vertial angle (0)
v_angle_min: Min vertical angle (-pi/2)
v_angle_step: Step by which each vertical angle increases
v_angle_count: Number of vertical scan lines (20)
range_min: Minimum range in which LiDAR will detect
range_max: Maximum range in which LiDAR will detect
(Else, it will return inf)
ranges: 2D array containing the sensed ranges (where the rows have the same vertical angle
and the columns have the same horizontal angle)
'''
def display_LiDAR(gazebo_sub):
    print("Compiling LiDAR result dictionary")

    # Timestamps
    sec, nsec = gazebo_sub.LaserScanStamped.time.sec, gazebo_sub.LaserScanStamped.time.nsec

    # Position and Orientation
    x_pos, y_pos, z_pos = gazebo_sub.LaserScanStamped.scan.world_pose.position.x, gazebo_sub.LaserScanStamped.scan.world_pose.position.y, gazebo_sub.LaserScanStamped.scan.world_pose.position.z
    x_ori, y_ori, z_ori, w_ori = gazebo_sub.LaserScanStamped.scan.world_pose.orientation.x, gazebo_sub.LaserScanStamped.scan.world_pose.orientation.y, gazebo_sub.LaserScanStamped.scan.world_pose.orientation.z, gazebo_sub.LaserScanStamped.scan.world_pose.orientation.w

    result = {'sec': sec, 'nsec': nsec, 'x_pos': x_pos, 'y_pos': y_pos, 'z_pos': z_pos, 'x_ori': x_ori, 'y_ori': y_ori, 'z_ori': z_ori, 'w_ori': w_ori}

    # Bounds
    result['h_angle_max'] = gazebo_sub.LaserScanStamped.scan.angle_max
    result['h_angle_min'] = -1 * result['h_angle_max']
    result['h_angle_step'] = gazebo_sub.LaserScanStamped.scan.angle_step
    result['h_angle_count'] = 20

    result['range_min'] = 0.2
    result['range_max'] = 10

    result['v_angle_max'] = 0
    result['v_angle_min'] = gazebo_sub.LaserScanStamped.scan.vertical_angle_min
    result['v_angle_step'] = gazebo_sub.LaserScanStamped.scan.vertical_angle_step
    result['v_angle_count'] = 9

    # Ranges
    ranges = []
    for v in range(1, result['v_angle_count'] + 1):
        row = []
        for h in range(1, result['h_angle_count'] + 1):
            row.append(gazebo_sub.LaserScanStamped.scan.ranges[(v * h) - 1])
        ranges.append(row)
    result['ranges'] = ranges

    return result

'''
Converts the data from the GPS sensor into a usable data structure
Argument: An instance of GazeboMessageSubscriber
Returns a dictionary with the following values:
sec: current time in seconds
nsec: current time in nanoseconds(?)
lat_deg: latitude given in degrees
long_deg: longitude are given in degrees
altitude: Not sure if this is relative or absolute
v_east: velocity in the east direction (positive longitude?)
v_north: velocity in the north direction (positive latitude?)
v_up: velocity upwards (towards space? away from the ground?)
'''
def display_GPS(gazebo_sub):
    print("Compiling GPS result dictionary")

    # Timestamps
    sec, nsec = gazebo_sub.GPS.time.sec, gazebo_sub.GPS.time.nsec

    result = {'sec': sec, 'nsec': nsec}

    # Position
    result['lat_deg'] = gazebo_sub.GPS.latitude_deg
    result['long_deg'] = gazebo_sub.GPS.longitude_deg
    result['altitude'] = gazebo_sub.GPS.altitude

    # Velocity
    result['v_east'] = gazebo_sub.GPS.velocity_east
    result['v_north'] = gazebo_sub.GPS.velocity_north
    result['v_up'] = gazebo_sub.GPS.velocity_up

    return result

'''
Retrieves the values from both the GPS and LiDAR sensor and returns it
Returns two dictionaries (make sure to use two variables to retrieve)
'''
async def retrieveSensorData():
    print('-- Retrieving LiDAR Sensor Data')
    lidar_val = await gz_sub.get_LaserScanStamped()
    return display_LiDAR(gz_sub)

'''
Retrieves initial GPS state of drone

What is the argument of the function??
Why make this a function if we just need to define it once?

I threw a bunch of global keywords here-will clean this up later
'''
def retrieveInitialState(lla_ref):
    # # Retrieves initial GPS data
    # initial_lidar_data, initial_gps_data = await retrieveSensorData()

    # Initializes global variables
    global current_lat, current_long, current_alt, unit_vector_lat, unit_vector_long, final_lat, final_long

    #initial points
    current_lat = lla_ref[0]
    current_long = lla_ref[1]
    current_alt = lla_ref[2]

    #ask for final point and AGL
    final_lat = lla_ref[0] 
    final_long = lla_ref[1] + 0.0004218

    #full vector
    full_lat = final_lat - current_lat
    full_long = final_long - current_long

    #magnitude of final vector
    magnitude = (full_lat ** 2 + full_long ** 2) ** 0.5

    #unit vector
    unit_vector_lat = full_lat / magnitude
    unit_vector_long = full_long / magnitude

'''
~Computational Analysis~
takes data and returns lat, long, and alt for next point

Arguments: The dictionary containing LiDAR values and the dictionary
containing GPS values and uses this to

Output: New latitude, longitude, altitude
'''
def computationalAnalysis(lidar_dict):
    # Iterate through gridline 10 until we reach a value that isn't infinite
    farthest_gridline = 8
    r2 = lidar_dict['ranges'][farthest_gridline][10]
    while (np.isnan(r2)):
        farthest_gridline -= 1
        r2 = lidar_dict['ranges'][farthest_gridline][10]

    #the grid lines we are using can change, fow now I just picked the bottom one and the other one step size away degrees away

    r1 = lidar_dict['ranges'][0][10]

    print(f"Gridlines - r1: {r1}, r2: {r2}")

    #variables
    alpha1 = lidar_dict['v_angle_min']
    alpha2 = alpha1 + lidar_dict['v_angle_step'] * farthest_gridline
    theta = lidar_dict['y_ori']

    print(f"Variables - alpha1: {alpha1}, alpha2: {alpha2}, theta: {theta}")

    #equations
    y1 = r1 * math.sin(theta + alpha1)
    x1 = r1 * math.cos(theta + alpha1)
    y2 = r2 * math.sin(theta + alpha2)
    x2 = r2 * math.cos(theta + alpha2)

    print(f"Equations - y1: {y1}, x1: {x1}, y2: {y2}, x2: {x2}")

    #change in values
    change_lat = unit_vector_lat * (x2 - x1) / 111000
    change_long = unit_vector_long * (x2 - x1) / 111000
    change_alt = y2 - y1

    print(f"Change in values - lat: {change_lat}, long: {change_long}, alt: {change_alt}")

    #redefine current values to goal
    global current_lat, current_long, current_alt
    current_lat += change_lat
    current_long += change_long
    current_alt += change_alt

    return current_lat, current_long, current_alt


async def run_mission(drone, mission_items, lla_ref, gz_sub):
    max_speed = 1 # m/s
    done = False # Signals end of mission

    mission_item_idx = 0 # Keeps track of the mission item we're on

    async for mission_progress in drone.mission.mission_progress():
        if (not mission_progress.current == -1):
            print(f"Mission progress: "
                f"{mission_progress.current}/"
                f"{mission_progress.total}")

            if (mission_progress.current == mission_progress.total - 1 and not done):
                print("-- Pause and clear mission")
                await drone.mission.pause_mission()
                await drone.mission.clear_mission()

                mission_item_idx = mission_progress.current

                print("-- Get LiDAR Readings")
                lidar = await retrieveSensorData()

                print("-- Making new mission plan")
                # Grabs the new lat/long/alt from the computaitonal analysis function
                new_lat, new_long, new_alt = computationalAnalysis(lidar)

                # Runs checks to see if the drone is done with the mission
                print("-- * Analysis Completed ", new_lat, new_long, new_alt)
                delta_lat = abs(final_lat - new_lat)
                delta_long = abs(final_long - new_long)
                distance_2d = math.sqrt(delta_lat * delta_lat + delta_long * delta_long)
                print("-- * Distance from end point calculated ", delta_lat, delta_long, distance_2d)
                if (distance_2d < .000045): # If the drone is close enough to the final waypoint (~5m), it proceeds to land
                    done = True
                    print("-- * Drone is close!")
                else: # Else, it inserts the waypoint with the new coords
                    print(f"-- * Injecting waypoint at {mission_item_idx}, Lat: {new_lat}, Long: {new_long}, Alt{new_alt}")
                    mission_items.insert(mission_item_idx, MissionItem(new_lat,
                                        new_long,
                                        new_alt,
                                        max_speed,
                                        True,
                                        float('nan'),
                                        float('nan'),
                                        MissionItem.CameraAction.NONE,
                                        float('nan'),
                                        float('nan')))

                mission_plan = MissionPlan(mission_items)

                print("-- Uploading updated mission")
                await drone.mission.upload_mission(mission_plan)
                await asyncio.sleep(1) # Lets upload finish

                print("-- Resuming mission")
                await drone.mission.set_current_mission_item(mission_item_idx)
                await drone.mission.start_mission()

        if (mission_progress.current == mission_progress.total and done):
            print("-- Landing")
            await drone.action.land()

async def run():
    # Apparently a test message makes subsequent requests faster
    gz_task = asyncio.create_task(gz_sub.connect())
    await gz_sub.get_LaserScanStamped()

    # Connects to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    print("Fetching amsl altitude at home location....")
    lla_ref = [] # Latitude, Longitude, Altitude reference
    async for terrain_info in drone.telemetry.home():
        lla_ref = [terrain_info.latitude_deg, terrain_info.longitude_deg, terrain_info.relative_altitude_m]
        break

    print("Retrieving intial GPS data...")
    retrieveInitialState(lla_ref)

    # Height above ground to maintain (3m)
    AGL = lla_ref[2] + 3

    # The mission items would be appended to this array here
    mission_items = []
    # mission_items.append(MissionItems(insert arguments here))

    # First waypoint would be directly above the starting location
    mission_items.append(MissionItem(lla_ref[0],
                                     lla_ref[1],
                                     AGL,
                                     2,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))

    # Last waypoint is the end [I'm assuming lat is y rn and lon is y] --> (38, 0, 1)
    mission_items.append(MissionItem(final_lat,
                                     final_long,
                                     AGL,
                                     2,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))

    mission_plan = MissionPlan(mission_items) # compiles the initial mission plan

    # "Spool up the mission and mission monitor"
    mission_task = asyncio.create_task(run_mission(drone, mission_items, lla_ref, gz_sub))
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, [mission_task, gz_task]))

    print("-- Arming")
    await drone.action.arm()

    print("awaiting")
    await asyncio.sleep(1)
    print("awaiting done")

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("awaiting")
    await asyncio.sleep(1)
    print("awaiting done")

    await drone.action.set_takeoff_altitude(3)

    print("-- Starting mission")
    await drone.mission.start_mission()
    await termination_task

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
