#!/usr/bin/env python3

import asyncio
import pygazebo
import pygazebo.msg.v11.laserscan_stamped_pb2 # Imports LiDAR readouts
import pygazebo.msg.v11.gps_pb2 # Imports GPS readouts
import math

from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)

# The gazebo master from PX4 message
HOST, PORT = "127.0.0.1", 11345

# If I'm not mistaken, this class utilizes protocol buffers to retrieve data from the sensors
# If we don't need GPS and LiDAR data simultaneously, I can split these into two classes
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

'''
Converts the data from the LiDAR sensor into a usable data structure

Argument: An instance of GazeboMessageSubscriber

Returns a dictionary with the following values:
sec: current time in seconds
nsec: current time in nanoseconds(?)

x_pos: x coord of drone
y_pos: y coord of drone
z_pos: z coord of drone

x_ori: roll
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

    slope = (y2 - y1) / (x2 - x1)
    beta = math.atan(slope)
    #if the slope is super small, pretend like its flat
    if abs(beta) < .001:
        beta = 0
    return beta


    # TODO: Do math or whatever

async def run():
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
    async for terrain_info in drone.telemetry.home():
        home_alt = terrain_info.relative_altitude_m
        home_lat = terrain_info.latitude_deg
        home_lon = terrain_info.longitude_deg
        break

    # The mission items would be appended to this array here
    mission_items = []
    # mission_items.append(MissionItems(insert arguments here))

    # Below is the demo mission path --> It's been commented out
    mission_items.append(MissionItem(home_lat + 0.0001,
                                     home_lon + 0.0000,
                                     home_alt + 5,
                                     10,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(home_lat + 0.0001,
                                     home_lon + 0.0001,
                                     home_alt + 5,
                                     10,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(home_lat + 0.0000,
                                     home_lon + 0.0001,
                                     home_alt + 5,
                                     10,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))
    mission_items.append(MissionItem(home_lat - 0.0001,
                                     home_lon + 0.0001,
                                     home_alt + 5,
                                     10,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))

    # This here is the code to inject a waypoint into the mission plan (?)
    inject_pt_task = asyncio.ensure_future(inject_pt(drone, mission_items, home_alt, home_lat, home_lon))
    running_tasks = [inject_pt_task]

    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))

    mission_plan = MissionPlan(mission_items)

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

    print("-- Starting mission")
    await drone.mission.start_mission()

    await termination_task

async def inject_pt(drone, mission_items, home_alt, home_lat, home_lon):
    pt_injected = False
    async for mission_progress in drone.mission.mission_progress():
        if(not mission_progress.current == -1):
            print(f"Mission progress: "
                f"{mission_progress.current}/"
                f"{mission_progress.total}")

            # This retrieves sensor data each time the drone reaches a waypoint
            # I'm pretty sure the final position of Computational Analysis
            print('* Running ~Computational Analysis~ *')
            lidar, gps = await retrieveSensorData()
            computationalAnalysis(lidar, gps)

            if(mission_progress.current == mission_progress.total and not pt_injected):
                mission_item_idx = mission_progress.current
                print("-- Pausing mission")
                await drone.mission.pause_mission()
                await drone.mission.clear_mission()

                print(f"-- Injecting waypoint at "
                f"{mission_item_idx}")

                mission_items.insert(mission_progress.current, MissionItem(home_lat,
                                     home_lon,
                                     home_alt,
                                     10,
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan')))
                mission_plan = MissionPlan(mission_items)
                await drone.mission.set_return_to_launch_after_mission(True)

                print("-- Uploading updated mission")
                await drone.mission.upload_mission(mission_plan)

                print("-- Resuming mission")
                await drone.mission.set_current_mission_item(mission_item_idx)
                await drone.mission.start_mission()

                pt_injected = True
        if(mission_progress.current == mission_progress.total):
            print("-- Landing")
            await drone.action.land()


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


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
