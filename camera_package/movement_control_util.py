import math
import time
import datetime


def calculate_speed(base_speed, time_on, wheel_distance, angle):
    speed_1 = ((2 * math.pi * ((base_speed * time_on) /
               (2 * math.pi) + wheel_distance))*(angle/360)) / time_on
    speed_2 = ((2 * math.pi * (base_speed * time_on) /
               (2 * math.pi))*(angle/360)) / time_on
    return speed_1, speed_2


def set_timestamp(list, set_first_time=True):
    current_time = datetime.datetime.now().time()
    if set_first_time:
        list[7] = current_time
    else:
        list[8] = current_time
    return list


def get_current_time():
    current_time = datetime.datetime.now().time()
    return current_time


def compare_times(list, old_time):
    time1 = list[7]        # time the image was shot
    time2 = old_time       # time the image was processed
    print("Time1:", time1, "Time2:", time2)
    # Parse the time strings into datetime objects
    if isinstance(time1, str):
        time1 = datetime.datetime.strptime(time1, "%H:%M:%S.%f").time()

    if isinstance(time2, str):
        time2 = datetime.datetime.strptime(time2, "%H:%M:%S.%f").time()

    old_time = list[8]     # time the new image was processed
    # Compare the times

    if time1 > time2:
        use_data = True
    else:
        use_data = False
    return use_data, old_time


def add_seconds_to_time(time1, milliseconds):
    if isinstance(time1, str):
        time1 = datetime.datetime.strptime(time1, "%H:%M:%S.%f").time()

    # Convert the time object to a datetime object
    now = datetime.datetime.now()
    datetime1 = datetime.datetime.combine(now, time1)
    milliseconds = milliseconds * 1000
    milliseconds_delta = datetime.timedelta(milliseconds=milliseconds)    # Create a timedelta object for the milliseconds
    new_datetime = datetime1 + milliseconds_delta   # Add the milliseconds to the datetime

    # Convert the datetime object back to a time object
    new_time = new_datetime.time()

    return new_time
