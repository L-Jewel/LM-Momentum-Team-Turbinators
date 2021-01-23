#!/usr/bin/env python3

import asyncio
import pygazebo
import pygazebo.msg.v11.laserscan_stamped_pb2 # Imports LiDAR readouts
import pygazebo.msg.v11.gps_pb2 # Imports GPS readouts
import math
import time

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
    result['h_angle_max'] = math.pi / 6
    result['h_angle_min'] = -1 * result['h_angle_max']
    result['h_angle_step'] = gazebo_sub.LaserScanStamped.scan.angle_step
    result['h_angle_count'] = 20

    result['range_min'] = 0.2
    result['range_max'] = 10

    result['v_angle_max'] = 0
    result['v_angle_mix'] = -1 * (math.pi / 2)
    result['v_angle_step'] = gazebo_sub.LaserScanStamped.scan.vertical_angle_step
    result['v_angle_count'] = 9

    # Ranges
    ranges = []
    for v in range(1, result['v_angle_count'] + 1):
        row = []
        for h in range(1, result['h_angle_count'] + 1):
            row.append(gazebo_sub.LaserScanStamped.scan.ranges[(result['v_angle_count'] * result['h_angle_count']) - 1])
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
    print('-- Retrieving LiDAR and GPS Sensor Data')
    gps_val = await gz_sub.get_GPS()
    lidar_val = await gz_sub.get_LaserScanStamped()
    return display_LiDAR(gz_sub), display_GPS(gz_sub)

'''
~Computational Analysis~
Arguments: The dictionary containing LiDAR values and the dictionary containing GPS values
'''

#function to get the initial point (only run once?)

def getInitialPoint():

    await gz_sub.get_GPS()
    latitude = display_GPS(gz_sub)['long_deg']
    longitude = display_GPS(gz_sub)['lat_deg']

    return latitude, longitude

#define the initial points and get the final point
initial_lat, initial_long = getInitialPoint()
final_lat = input("What is the final latitude?\n")
final_long = input("What is the final longitude?\n")

class FinalVector:
    def __init__(self, lat, long, alt):
        self.lat = final_lat - initial_lat
        self.long = final_long - initial_long
        self.magnitude = (self.lat ** 2 + self.long ** 2) ** 0.5
        #unit vector
        self.vector = (self.lat / self.magnitude, self.long / self.magnitude)

def computationalAnalysis(lidar_dict, gps_dict):
    #variables
    alpha1 = math.pi / -2
    alpha2 = -7 / 18 * math.pi
    theta = lidar_dict['y_ori']
    #the grid lines we are using can change, fow now I just picked the bottom one and one 20 degrees away
    r1 = lidar_dict['ranges'][9][10]
    r2 = lidar_dict['ranges'][7][10]
    #equations
    y1 = r1 * math.sin(theta + alpha1)
    x1 = r1 * math.cos(theta + alpha1)
    y2 = r2 * math.sin(theta + alpha2)
    x2 = r2 * math.cos(theta + alpha2)

    slope = (y2 - y1) / (x2 - x1) * 111000
    beta = math.atan(slope)
    print(f"this is the angle of inclination: {beta}")
    return beta



def motionPlanning(beta):
    #need all the calculations and for the funciton to yield values
    #for the run function to go to based on the angle


    print(beta)

    # TODO: Do math or whatever
    return # Return three different values: new_lat, new_long, new_alt

async def run_mission(drone, mission_items, lla_ref, gz_sub):
    max_speed = 2 # m/s
    done = False # Signals end of mission

    mission_item_idx = 0 # Keeps track of the mission item we're on
    print("made it")

    async for mission_progress in drone.mission.mission_progress():
        # breakpoint()
        print(mission_progress)
        if (not mission_progress.current == -1):
            print(f"Mission progress: "
                f"{mission_progress.current}/"
                f"{mission_progress.total}")

            if (mission_progress.current == mission_progress.total - 1 and not done):
                print("-- Pause and clear mission")
                await drone.mission.pause_mission()
                await drone.mission.clear_mission()

                mission_item_idx = mission_progress.current

                print("-- Get LiDAR and GPS Readings")
                lidar, gps = await retrieveSensorData()
                # computationalAnalysis(lidar, gps)
                # TODO: Algorithm boyo goes here
                # When the drone is ready to land, set done equal to True
                if (mission_progress.current == len(mission_items) - 1):
                    done = True

                print("-- Making new mission plan")
                mission_items.insert(mission_item_idx, MissionItem(lla_ref[0],
                                    lla_ref[1] + 0.0001,
                                    lla_ref[2] + 1,
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

    AGL = lla_ref[2] + 1 # Height above ground to maintain

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

    # Last waypoint is the end
    final_lat = lla_ref[0] + 0.0001
    final_lon = lla_ref[1] + 0.0001
    mission_items.append(MissionItem(final_lat,
                                     final_lon,
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

    await drone.action.set_takeoff_altitude(AGL)

    print("-- Starting mission")
    await drone.mission.start_mission()
    print('yeet')
    await termination_task

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
