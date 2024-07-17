import machine
import time
import gc
import math

machine.freq(125000000)
print("Machine is runnin at: ", machine.freq())

led = machine.Pin(25, machine.Pin.OUT)
i2c = machine.I2C(0, sda=20, scl=21, freq=400000)
uart = machine.UART(0, baudrate=115200, tx=machine.Pin(16), rx=machine.Pin(17))
# Input joystick
CH1 = machine.Pin(13, machine.Pin.IN)  # Roll
CH2 = machine.Pin(11, machine.Pin.IN)  # Pitch
CH3 = machine.Pin(9, machine.Pin.IN)  # Throttle
CH4 = machine.Pin(7, machine.Pin.IN)  # Yaw
# Output Motor
gpio_motor1 = 2  # front left, clockwise
gpio_motor2 = 3  # front right, counter clockwise
gpio_motor3 = 4  # rear left, counter clockwise
gpio_motor4 = 5  # rear right, clockwise

alpha = 0.9
lp_alpha = 0.4

#print(machine.freq())

for x in range(8):
    led.on()
    time.sleep(0.1)
    led.off()
    time.sleep(0.1)

def sigmoid(x, slope):
    """
    Convert input x to sigmoid curve.
    
    Parameters:
    - x: Input value to transform.
    - slope: Slope parameter of the sigmoid curve.
    
    Returns:
    - Transformed value according to sigmoid curve.
    """
    return 1000 / (1 + math.exp(-slope * x))


def combine_register_values(h, l):
    if not h[0] & 0x80:
        return h[0] << 8 | l[0]
    return -((h[0] ^ 255) << 8) | (l[0] ^ 255) + 1


def calculate_roll_pitch(accel_data, gyro_data, dt, prev_roll, prev_pitch):
    acc_x, acc_y, acc_z = accel_data
    gyro_x, gyro_y, gyro_z = gyro_data

    # Roll calculation using accelerometer data
    roll_acc = math.atan2(acc_y, acc_z) * 180 / math.pi

    # Pitch calculation using accelerometer data
    pitch_acc = (
        math.atan2(-acc_x, math.sqrt(acc_y * acc_y + acc_z * acc_z)) * 180 / math.pi
    )

    # Gyro integration
    roll_gyro = prev_roll + (gyro_x) * dt
    pitch_gyro = prev_pitch + (gyro_y) * dt

    # Combine gyro and accelerometer data using complementary filter
    roll = (alpha * roll_gyro + (1 - alpha) * roll_acc)
    pitch = (alpha * pitch_gyro + (1 - alpha) * pitch_acc)

    return roll, pitch



MPU6050_ADDR = 0x68
i2c.writeto_mem(MPU6050_ADDR, 0x6B, bytes([0x01]))  # wake it up
i2c.writeto_mem(MPU6050_ADDR, 0x1A, bytes([0x05]))  # set low pass filter to 5 (0-6)
i2c.writeto_mem(MPU6050_ADDR, 0x1C, bytes([0x10]))  # acceleration config
i2c.writeto_mem(MPU6050_ADDR, 0x1B, bytes([0x08]))  # set gyro scale to 1 (0-3)

# confirm IMU is set up
whoami: int = i2c.readfrom_mem(MPU6050_ADDR, 0x75, 1)[0]
lpf: int = i2c.readfrom_mem(MPU6050_ADDR, 0x1A, 1)[0]
gs: int = i2c.readfrom_mem(MPU6050_ADDR, 0x1B, 1)[0]

# did who am I work?
if whoami == 104:  # 0x68
    print("MPU-6050 WHOAMI validated!")
else:
    print("ERROR! MPU-6050 WHOAMI failed! '" + str(whoami) + "' returned.")

# did lpf get set?
if lpf == 0x05:
    print("MPU-6050 LPF set to " + str(lpf) + " correctly.")
else:
    print("ERROR! MPU-6050 LPF did not set correctly. Set to '" + str(lpf) + "'")

# did gyro scale get set?
if gs == 0x08:
    print("MPU-6050 Gyro Scale set to " + str(gs) + " correctly.")
else:
    print("ERROR! MPU-6050 gyro scale did not set correctly. " + str(gs) + " returned.")

gyro_bias_x = 0.55
gyro_bias_y = 0.91
gyro_bias_z = 1.05

accel_bias_x = -0.1002
accel_bias_y = -0.047
accel_bias_z = -0.0946


"""print("CALIBRATING ROLL AND PITCH!")
roll_bias_list: list[float] = []
pitch_bias_list: list[float] = []
prev_time: int = time.ticks_ms()
prev_roll = 0
prev_pitch = 0
for i in range(200):
    dt = (time.ticks_ms() - prev_time) / 1000 
    start_time = time.ticks_ms()
    # gathering inputs
    accelerometer_data: list[float] = [(combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x3B, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x3C, 1)) / 4096) - accel_bias_x,
                                       (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x3D, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x3E, 1)) / 4096) - accel_bias_y,
                                       (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x3F, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x40, 1)) / 4096) - accel_bias_z]
    gyro_data: list[float] = [(combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x43, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x44, 1),) / 65.5) - gyro_bias_x,
                              (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x45, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x46, 1),) / 65.5) - gyro_bias_y,
                              (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x47, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x48, 1),) / 65.5) - gyro_bias_z]
    
    roll, pitch = calculate_roll_pitch(accelerometer_data, gyro_data, dt, prev_roll, prev_pitch)
    prev_roll = roll
    prev_pitch = pitch
    roll_bias_list.append(roll)
    pitch_bias_list.append(pitch)
    time.sleep(0.025)
roll_bias = sum(roll_bias_list) / 2000
pitch_bias = sum(pitch_bias_list) / 2000
print("Roll_bias: ", roll_bias, " Pitch_bias: ", pitch_bias)
    

# Deleting useless data from memory
print("Free memory:", gc.mem_free())
print("Allocated memory:", gc.mem_alloc())
del roll_bias_list, pitch_bias_list, accelerometer_data, gyro_data
gc.collect()
print("Allocated memory:", gc.mem_alloc())"""

# min and max throttle (nanoseconds)
throttle_max: int = 2000000
throttle_min: int = 1000000

# Set up PWM's
M1: machine.PWM = machine.PWM(machine.Pin(gpio_motor1))
M2: machine.PWM = machine.PWM(machine.Pin(gpio_motor2))
M3: machine.PWM = machine.PWM(machine.Pin(gpio_motor3))
M4: machine.PWM = machine.PWM(machine.Pin(gpio_motor4))
M1.freq(250)
M2.freq(250)
M3.freq(250)
M4.freq(250)
print("Motor PWM's set up @ 250 hz")

M1.duty_ns(throttle_min)
M2.duty_ns(throttle_min)
M3.duty_ns(throttle_min)
M4.duty_ns(throttle_min)
print("ESC Armed And Ready")


prev_time = time.ticks_ms()
# Initial angles
roll: float = 0
pitch: float = 0

# Initializing pid
kp_roll: float = 3.0
ki_roll: float = 0.2
kd_roll: float = 0.01

kp_pitch: float = 3.0
ki_pitch: float = 0.2
kd_pitch: float = 0.01

kp_yaw: float = 3.0
ki_yaw: float = 0.2
kd_yaw: float = 0.01

# Initializing pid variables
roll_integral = 0.0
previous_roll_error = 0.0

pitch_integral = 0.0
previous_pitch_error = 0.0

yaw_integral = 0.0
previous_yaw_error = 0.0

iteration = 0
prev_pitch = 0 
prev_roll = 0

prev_gyro_x = 0
prev_gyro_y = 0
prev_gyro_z = 0
while True:
    current_time = time.ticks_ms()
    dt = (current_time - prev_time) / 1000 # Convert to seconds
    prev_time = current_time

    receiver_data = [machine.time_pulse_us(CH1, 1, 30000),
                     machine.time_pulse_us(CH2, 1, 30000),
                     machine.time_pulse_us(CH3, 1, 30000),
                     machine.time_pulse_us(CH4, 1, 30000)]
    
    # for testing
    #receiver_data[2] = 1011
    
    # ADD IF STATEMENT FOR FLIGHT STATE
    if receiver_data[2] == (-2):
        led.on()
        print("Transmitter is turned off!")
        time.sleep(2)
        led.off()

    elif receiver_data[2] <= 1010:
        led.on()
        #getting pid values from esp8266
        if uart.any():
            received_data = uart.read()
            received_data = received_data.decode("utf-8").lstrip("hii")
            received_data = [float(num) for num in received_data.split(",")]
            kp_pitch: float = received_data[0]
            kd_pitch: float = received_data[1]
            ki_pitch: float = received_data[2]
            print("Data received from esp: ", received_data)

        else:
            print("No Data from esp")

        M1.duty_ns(throttle_min)
        M2.duty_ns(throttle_min)
        M3.duty_ns(throttle_min)
        M4.duty_ns(throttle_min)
        # Initializing pid variables
        roll_integral = 0.0
        previous_roll_error = 0.0

        pitch_integral = 0.0
        previous_pitch_error = 0.0

        yaw_integral = 0.0
        previous_yaw_error = 0.0
        print("Throttle is off: ", receiver_data)
        time.sleep(1)
        led.off()

    elif receiver_data[2] > 1010:
        led.on()
        #New_value = ((old_value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min
        ch1_value = ((receiver_data[0] - 1000) / (2000 - 1000)) * (60 - (-60)) + (-60)
        ch1_value = int(max(-60, min(ch1_value, 60)))
        ch2_value = ((receiver_data[1] - 1000) / (2000 - 1000)) * (60 - (-60)) + (-60)
        ch2_value = int(max(-60, min(ch2_value, 60)))
        ch3_value = ((receiver_data[2] - 1018) / (1833 - 1018)) * (800 - (0)) + (0)
        ch3_value = max(0, min(ch3_value, 800))
        ch4_value = ((receiver_data[3] - 1000) / (2000 - 1000)) * (60 - (-60)) + (-60)
        #ch4_value = ((ch4_value - 0) / (1000 - 0)) * (60 - (-60)) + (-60)
        ch4_value = int(max(-60, min(ch4_value, 60)))
        #print("CH4:", receiver_data[3], "    ch4 value:", ch4_value)
        #print(receiver_data)

        

        # gathering inputs
        accelerometer_data: list[float] = [(combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x3B, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x3C, 1)) / 4096) - (-0.01359884),
                                            (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x3D, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x3E, 1)) / 4096) - (-0.01445438),
                                            (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x3F, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x40, 1)) / 4096)  - (-0.09077817)]
        gyro_data: list[float] = [(combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x43, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x44, 1),) / 65.5) - 0.6030666,
                                (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x45, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x46, 1),) / 65.5) - 0.9627455,
                                (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x47, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x48, 1),) / 65.5) - 1.057611]
        
        roll, pitch = calculate_roll_pitch(accelerometer_data, gyro_data, dt, roll, pitch)
        #smoothed_pitch = lp_alpha * prev_pitch + (1 - lp_alpha) * pitch
        #smoothed_roll = lp_alpha * prev_roll + (1 - lp_alpha) * roll
        prev_roll = roll
        prev_pitch = pitch
        roll_corrected = roll
        pitch_corrected = pitch

        # Calculating Pid
        roll_error = ch1_value - roll_corrected
        pitch_error = ch2_value - pitch_corrected # error = desired_value - actual_value
        yaw_error = ch4_value - gyro_data[2]

        # Proportional term
        P_roll = kp_roll * roll_error
        P_pitch = kp_pitch * pitch_error
        P_yaw = kp_yaw * yaw_error

        # Integral term
        roll_integral += roll_error
        roll_integral = max(-100, min(roll_integral, 100))
        pitch_integral += pitch_error
        pitch_integral = max(-100, min(pitch_integral, 100))
        yaw_integral += yaw_error
        yaw_integral = max(-100, min(yaw_integral, 100))
        #print(roll_integral, pitch_integral)
        
        I_roll = ki_roll * roll_integral
        I_pitch = ki_pitch * pitch_integral
        I_yaw = ki_yaw * yaw_integral

        # Derivative term
        roll_derivative = roll_error - previous_roll_error
        pitch_derivative = pitch_error - previous_pitch_error
        yaw_derivative = yaw_error - previous_yaw_error

        D_roll = kd_roll * roll_derivative
        D_pitch = kd_pitch * pitch_derivative
        D_yaw = kd_yaw * yaw_derivative

        # PID output
        pid_output_roll = max(-200, min((P_roll + I_roll + D_roll), 200))
        pid_output_pitch = max(-200, min((P_pitch + I_pitch + D_pitch), 200))
        pid_output_yaw = max(-200, min((P_yaw + I_yaw + D_yaw), 200))

        # Update previous error
        previous_roll_error = pid_output_roll
        previous_pitch_error = pid_output_pitch
        previous_yaw_error = pid_output_yaw

        #print("Roll: ", roll_corrected, "Pitch: ", pitch_corrected)
        #print(receiver_data)
        #print("Pid Pitch:", pid_output_pitch, "    Pid Roll:", pid_output_roll, "    Pid Yaw:", pid_output_yaw)
        #print("Pid Pitch:", pid_output_pitch, "    Pitch Error:", pitch_error, "    Pitch Angle:", pitch_corrected, "    Pitch setpoint:", ch2_value)
        #print("Pid Yaw:", pid_output_yaw, "    Yaw Error:", yaw_error, "    Yaw Angle Rate:", gyro_data[2], "    Yaw setpoint:", ch4_value)
        #print("Pid Roll:", pid_output_roll, "    Roll Error:", roll_error, "    Roll Angle:", roll_corrected, "    Roll setpoint:", ch1_value)
        #print("Roll Error: ", roll_error, "Pid roll: ", pid_output_roll,"Pitch error: ", pitch_error, " Pid pitch: ", pid_output_pitch, "ch1value: ", ch1_value, "rollCorrected: ", roll_corrected)
        #print(ch1_value, " ", ch2_value, " ", ch3_value, " ", ch4_value)
        motor_mix1 = ch3_value + pid_output_roll - pid_output_pitch - pid_output_yaw # front left, clockwise
        motor_mix1 = max(0, min(motor_mix1, 1000))
        motor_mix2 = ch3_value - pid_output_roll - pid_output_pitch + pid_output_yaw # front right, counter clockwise
        motor_mix2 = max(0, min(motor_mix2, 1000))
        motor_mix3 = ch3_value + pid_output_roll + pid_output_pitch + pid_output_yaw # rear left, counter clockwise
        motor_mix3 = max(0, min(motor_mix3, 1000))
        motor_mix4 = ch3_value - pid_output_roll + pid_output_pitch - pid_output_yaw # rear right, clockwise
        motor_mix4 = max(0, min(motor_mix4, 1000))
        #print(motor_mix1, motor_mix2, motor_mix3, motor_mix4)

        # feeding output to the motors
        #New_value = ((old_value - old_min) / (old_max - old_min)) * (new_max - new_min) + new_min
        M1_duty_cycle = int(((motor_mix1 - 0) / (1000 - 0)) * (throttle_max - throttle_min) + throttle_min)
        M2_duty_cycle = int(((motor_mix2 - 0) / (1000 - 0)) * (throttle_max - throttle_min) + throttle_min)
        M3_duty_cycle = int(((motor_mix3 - 0) / (1000 - 0)) * (throttle_max - throttle_min) + throttle_min)
        M4_duty_cycle = int(((motor_mix4 - 0) / (1000 - 0)) * (throttle_max - throttle_min) + throttle_min)
        M1.duty_ns(M1_duty_cycle)
        M2.duty_ns(M2_duty_cycle)
        M3.duty_ns(M3_duty_cycle)
        M4.duty_ns(M4_duty_cycle)
        #print("Pitch Error:", pitch_error, "Pid_pitch:", pid_output_pitch)
        #print(M1_duty_cycle, M2_duty_cycle, M3_duty_cycle, M4_duty_cycle)

    else:
        led.on()
        print("Error! CH3 input: ", receiver_data[2])
        time.sleep_ms(500)
        led.off()


    #time.sleep(0.02)
    #print(dt)
    
    