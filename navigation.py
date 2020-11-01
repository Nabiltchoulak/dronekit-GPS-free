#!/usr/bin/env python
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time
import math
import matplotlib.pyplot as plt
import pandas as pd

########### constatnts of the journey

record = []
pitch_record = []
path_duration = 5
path = [(-0.9, 0.1), (-1,1), (-1.9, 1),(-3, 0.1),(-4,1),(-5,2)]

# Connect to the Vehicle simulator via the udp port

vehicle = connect("udp:127.0.0.1:14551", wait_ready=True)

#Pick coordinates at each moment to compare our results with the ground-truth data

def coordinates_to_distance(lon_1=0, lat_1=0, lon_2=0, lat_2=0):
    #this functon is returning the distance between two points from their respective coordinates
    a = 0
    c = 0
    R = 6371e3
    phi_1 = math.radians(lat_1)
    phi_2 = math.radians(lat_2)
    delta_phi = math.radians(lat_2 - lat_1)
    delta_lam = math.radians(lon_2 - lon_1)
    a = math.pow(math.sin(delta_phi * 0.5), 2) + math.cos(phi_1) * math.cos(phi_2) * math.pow(math.sin(delta_lam * 0.5),
                                                                                              2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    distance = R * c
    return distance


def arm_and_takeoff_nogps(aTargetAltitude):
    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        # print(" Altitude: %f  Desired: %f" %(current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude * 0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust=thrust)
        time.sleep(0.2)


def send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        1,  # Target system
        1,  # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
        0,  # Body roll rate in radian
        0,  # Body pitch rate in radian
        math.radians(yaw_rate),  # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)


def set_attitude(roll_angle=0.0, pitch_angle=0.0,
                 yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                 thrust=0.5, step=-1, duration=0, speed_record=[], pitch_record=[]):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
        # Uncomment to register speed values
        # speed_record[step].append(vehicle.groundspeed)
        if step >= 0:
            # print(" Groundspeed: %s" % vehicle.groundspeed)
            # print(" Pitch: %s" % vehicle.attitude.pitch)
            speed_record.append(vehicle.groundspeed)
            pitch_record.append(vehicle.attitude.pitch)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, pitch_angle,
                         0, 0, True,
                         thrust)


def yaw_from_coordinates(x1=0, y1=0, x2=0, y2=0):
    theta = math.degrees(math.atan2(x2 - x1, y2 - y1))
    if y2 - y1 < 0:
        theta += 90
    return theta


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

# Take off 2.5m in GUIDED_NOGPS mode.
arm_and_takeoff_nogps(2.5)


print("Hold position for 1 seconds")
set_attitude(yaw_angle=0,duration=1)
# Hold the initial position for 1 seconds to stabilize.
first = True
diff = []

for point in path:
    if first:
        # Stabilize the drone to angle 0° in the first waypoint
        yaw = yaw_from_coordinates(0, 0, point[0], point[1])
        first=False
    else:
        yaw = yaw_from_coordinates(current[0],current[1],point[0],point[1])
    # Uncomment this to obtain ground truth current position from SITL simulator
    #start_pos = vehicle.location.global_frame
    # Pitch record argument is used to observe pitch values over the flight
    set_attitude(yaw_angle=yaw, step=2, pitch_angle=-2.4, speed_record=record, pitch_record=pitch_record,
                 thrust=0.5, duration=3)

    # Uncomment this to obtain ground truth position reached from SITL simulator
    #end = vehicle.location.global_frame

    # This is used to evaluate the distance traveled to tune and test the technique from ground truth positions
    #real_distance = coordinates_to_distance(start_pos.lon, start_pos.lat, end.lon, end.lat)


    #Check ground truth values of speed
    #speeds = pd.Series(record)
    #speed_mean = speeds.mean()



    time.sleep(0.5)
    #Send this command to stabilize communication with SITL
    send_attitude_target(0, 0,
                         0, 0, True,
                         0.4)

#pitches = pd.Series(pitch_record)
print("Setting LAND mode...")
vehicle.mode = VehicleMode("LAND")
time.sleep(1)
# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.

print("Completed")

