import math
import time
import datetime

def calculate_speed_variable_time(base_rpm, radius, angle, wheel_distance, wheel_radius, correction_factor):    
    
    #base_rpm rpm
    #radius cm
    #angle °
    #wheel_distance cm
    #wheel_radius cm
    #base_speed cm/s

    base_speed = ((2 * math.pi * wheel_radius) / 60) * base_rpm

    if angle < 0:
        angle = -angle
        switch = True
    else:
        switch = False
    
    distance_in = (2 * math.pi * (radius) * (angle / 360))
    distance_out = (2 * math.pi * (radius + wheel_distance) * (angle / 360))

    time_out = distance_in / base_speed
    
    if distance_out == 0:
        rpm_out = base_rpm
        time_out = 1
    else:
        rpm_out = distance_out / time_out / (0.1047 * wheel_radius)
    rpm_in = base_rpm
    # print("rpm_out:", int(rpm_out), "rpm_in:", int(rpm_in), "time_out:", time_out)

    time_out = time_out * correction_factor
    
    if switch:
        return rpm_in, rpm_out, time_out
    else:    
        return rpm_out, rpm_in, time_out


def calculate_movement_variable_time(self, base_rpm, angle, move = False):
    
    radius = self.radius
    wheel_distance = self.wheel_distance
    wheel_radius = self.wheel_radius
    correction_factor = self.correction_factor

    if base_rpm > 0 and move == True:
        speed_out, speed_in, time_out = calculate_speed_variable_time(base_rpm, radius, angle, wheel_distance, wheel_radius, correction_factor)
        speed_right = speed_in
        speed_left = speed_out

    elif base_rpm < 0 and move == True:
        speed_out, speed_in, time_out = calculate_speed_variable_time(base_rpm, radius, angle, wheel_distance, wheel_radius, correction_factor)
        speed_right = speed_out
        speed_left = speed_in

    elif base_rpm != 0 and move == False:
        radius_2 = (wheel_distance / 2)
        wheel_distance_2 = -wheel_distance
        speed_out, speed_in, time_out = calculate_speed_variable_time(base_rpm, radius_2, angle, wheel_distance_2, wheel_radius, correction_factor)
        speed_right = -speed_in
        speed_left = -speed_out

    elif move == True:
        speed_right = base_rpm
        speed_left = base_rpm
        time_out = 1
    
    else:
        speed_right = 0
        speed_left = 0
        time_out = 0
    
    return int(speed_right), int(speed_left), time_out

def calculate_angle(self, servo_pan = True):
    # Winkelberechnung
    self.winkel_x = int((self.coordinate_x/(self.max_x))*self.max_winkel_x)
    self.winkel_y = int((self.coordinate_y/(self.max_y))*self.max_winkel_y)
    self.get_logger().info("Angle: [{}, {}]".format(self.winkel_x, self.winkel_y))

    self.servo_msg_hold[0] = self.servo_msg_hold[0] + self.winkel_x
    if servo_pan:
        self.servo_msg_hold[1] = self.servo_msg_hold[1] + self.winkel_y
    else:
        self.servo_msg_hold[1] = 0
    
    return self.servo_msg_hold, self.winkel_x, self.winkel_y


def determine_percentage_of_height(self):
    max_hight = round(self.distance_to_person * math.tan(math.radians(self.max_winkel_y)))
    return round((self.hight_of_person/max_hight) * 100)

def aproximate_distance(self, lenght_y):
    hight_percentage = (lenght_y / self.max_y)
    return int((hight_percentage / self.optimal_hight_percentage) * self.distance_to_person)



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


def compare_times(time1, time_old, old_time):
    time1 = time1        # time the image was shot
    time2 = old_time       # time the image was processed
    # Parse the time strings into datetime objects
    if isinstance(time1, str):
        time1 = datetime.datetime.strptime(time1, "%H:%M:%S.%f").time()

    if isinstance(time2, str):
        time2 = datetime.datetime.strptime(time2, "%H:%M:%S.%f").time()

    # Compare the times
    if time1 > time2:
        old_time = time_old     # time the new image was processed
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

def datetime_to_combined_int(dt):
    hours_minutes_combined = dt.hour * 100 + dt.minute
    seconds_milliseconds_combined = dt.second * 1000 + dt.microsecond // 1000
    return hours_minutes_combined, seconds_milliseconds_combined

def combined_int_to_datetime(hours_minutes_combined, seconds_milliseconds_combined):
    hours = hours_minutes_combined // 100
    minutes = hours_minutes_combined % 100
    seconds = seconds_milliseconds_combined // 1000
    milliseconds = (seconds_milliseconds_combined % 1000) * 1000
    time_ret = "{:02d}:{:02d}:{:02d}.{:03d}".format(hours, minutes, seconds, milliseconds)
    return time_ret