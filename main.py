import machine
import time
import gc
import math
import _thread

machine.freq(250000000)
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

lp_alpha = 0.4

set_gyro_angles = False
#print(machine.freq())

for x in range(12):
    led.on()
    time.sleep(0.1)
    led.off()
    time.sleep(0.1)

def sigmoid(x, slope):
    return 1000 / (1 + math.exp(-slope * x))

def reading_receiver_data():
    global receiver_data
    while True:
        receiver_data = [machine.time_pulse_us(CH1, 1, 30000),
                        machine.time_pulse_us(CH2, 1, 30000),
                        machine.time_pulse_us(CH3, 1, 30000),
                        machine.time_pulse_us(CH4, 1, 30000)]
        time.sleep_ms(2)


def combine_register_values(h, l):
    if not h[0] & 0x80:
        return h[0] << 8 | l[0]
    return -((h[0] ^ 255) << 8) | (l[0] ^ 255) + 1


def calculate_roll_pitch(accel_data, gyro_data, dt, prev_roll, prev_pitch):
    global set_gyro_angles

    acc_x, acc_y, acc_z = accel_data
    gyro_x, gyro_y, gyro_z = gyro_data

    # Gyro integration
    roll_gyro = prev_roll + (gyro_x) * dt
    pitch_gyro = prev_pitch + (gyro_y) * dt

    # Adjust the roll and pitch values based on the yaw rate
    roll_gyro += pitch_gyro * math.sin(math.radians(gyro_z * dt))
    pitch_gyro -= roll_gyro * math.sin(math.radians(gyro_z * dt))

    # Calculate the total accelerometer vector
    acc_total_vector = math.sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z))

    # Roll calculation using accelerometer data
    roll_acc = math.asin(float(acc_y) / acc_total_vector) * 57.296

    # Pitch calculation using accelerometer data
    pitch_acc = math.asin(float(acc_x) / acc_total_vector) * -57.296

    # Correct the drift of the gyro with the accelerometer data
    if set_gyro_angles:
        roll = roll_gyro * 0.999 + roll_acc * 0.001
        pitch = pitch_gyro * 0.999 + pitch_acc * 0.001
    else:
        roll = roll_acc
        pitch = pitch_acc
        set_gyro_angles = True

    # clamping down 
    #roll = (0.9 * prev_roll + 0.1 * roll)
    #pitch = (0.9 * prev_pitch + 0.1 * pitch)

    return roll, pitch

class PID:
    def __init__(self, kp, ki, kd, rate=1):
        self.kp: float = kp
        self.ki: float = ki
        self.kd: float = kd
        self.rate = rate
        self.integrator_min = -400
        self.integrator_max = 400
        self.output_max = 400
        self.prev_error = 0
        self.integral  = 0
        self.output = 0

    def update(self, setpoint, measured_value, dt):
        error = (setpoint - measured_value) * self.rate
        if self.output < 400 and self.output > -400:
            self.integral += error * dt

        # Clamp integral term
        self.integral = max(self.integrator_min, min(self.integrator_max, self.integral))

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # Now calculating the proportional integral derivative
        self.output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # Clamp PID output
        self.output = min(self.output, self.output_max)

        return self.output

def calibrate_gyro(i2c, addr=0x68, num_samples=1000):
    """Calibrate the gyroscope by calculating the bias values for each axis."""
    gyro_bias: list[float] = [0, 0, 0]
    
    print("Calibrating gyroscope...")
    
    for _ in range(num_samples):
        gyro_data = [(combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x43, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x44, 1),) / 65.5),
                    (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x45, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x46, 1),) / 65.5),
                    (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x47, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x48, 1),) / 65.5)]
        for i in range(3):
            gyro_bias[i] += gyro_data[i]
        time.sleep(0.01)  # Sleep for 10ms between readings
    
    for i in range(3):
        gyro_bias[i] /= num_samples
    
    print("Gyroscope calibration complete.")
    print("Gyro Bias X:", gyro_bias[0])
    print("Gyro Bias Y:", gyro_bias[1])
    print("Gyro Bias Z:", gyro_bias[2])
    
    return gyro_bias

"""def calibrate_accel(i2c, addr=0x68, num_samples=1000):
    accel_bias: list[float] = [0, 0, 0]
    
    print("Calibrating gyroscope...")
    
    for _ in range(num_samples):
        accel_data = [(combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x3B, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x3C, 1)) / 4096),
                        (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x3D, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x3E, 1)) / 4096),
                        (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x3F, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x40, 1)) / 4096) ]
        for i in range(3):
            accel_bias[i] += accel_data[i]
        time.sleep(0.01)  # Sleep for 10ms between readings
    
    for i in range(3):
        accel_bias[i] /= num_samples
    
    print("Gyroscope calibration complete.")
    print("accel Bias X:", accel_bias[0])
    print("accel Bias Y:", accel_bias[1])
    print("accel Bias Z:", accel_bias[2])
    
    return accel_bias"""


MPU6050_ADDR = 0x68
i2c.writeto_mem(MPU6050_ADDR, 0x6B, bytes([0x01]))  # wake it up
i2c.writeto_mem(MPU6050_ADDR, 0x1A, bytes([0x06]))  # set low pass filter to 5 (0-6)
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
if lpf == 0x06:
    print("MPU-6050 LPF set to " + str(lpf) + " correctly.")
else:
    print("ERROR! MPU-6050 LPF did not set correctly. Set to '" + str(lpf) + "'")

# did gyro scale get set?
if gs == 0x08:
    print("MPU-6050 Gyro Scale set to " + str(gs) + " correctly.")
else:
    print("ERROR! MPU-6050 gyro scale did not set correctly. " + str(gs) + " returned.")

gyro_bias = calibrate_gyro(i2c)
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
acro_mode = True
if acro_mode == True:
    kp_r_p = 3.1003
    ki_r_p = 0.25398
    kd_r_p = 0.13005
    #pid_roll = PID(kp=3.1003, ki=0.25398, kd=0.13005)
    pid_roll = PID(kp=kp_r_p, ki=ki_r_p, kd=kd_r_p)
    pid_pitch = PID(kp=kp_r_p, ki=ki_r_p, kd=kd_r_p)
    pid_yaw = PID(kp=3.15, ki=0.499, kd=0, rate=5)
else:
    kp_r_p = 15.502
    ki_r_p = 0.25398
    kd_r_p = 0.95
    pid_roll = PID(kp=kp_r_p, ki=ki_r_p, kd=kd_r_p)
    pid_pitch = PID(kp=kp_r_p, ki=ki_r_p, kd=kd_r_p)
    pid_yaw = PID(kp=3.15, ki=0.499, kd=0, rate=1)
prev_pitch = 0
prev_roll = 0

rc_time = 0
global receiver_data
receiver_data = [0,0,0,0]
second_thread = _thread.start_new_thread(reading_receiver_data, ())
while True:
    current_time = time.ticks_ms()
    dt = time.ticks_diff(current_time, prev_time) / 1000 # Convert to seconds
    #print(dt)
    prev_time = current_time
    
    #print(receiver_data)
    
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
        #getting pid values from esp8266
        if uart.any():
            received_data = uart.read()
            received_data = received_data.decode("utf-8").lstrip("hii")
            
            print("Raw received data:", received_data)  # Print raw data to inspect it
            
            try:
                received_data = [float(num) for num in received_data.split(",")]
                
                pid_pitch.kp = received_data[0]
                pid_roll.kp = received_data[0]
                pid_pitch.ki = received_data[1]
                pid_roll.ki = received_data[1]
                pid_pitch.kd = received_data[2]
                pid_roll.kd = received_data[2]
                
                print("Data received from esp:", received_data)
            except ValueError as e:
                print("Error in converting received data to float:", e)


        else:
            print("No Data from esp")

        M1.duty_ns(throttle_min)
        M2.duty_ns(throttle_min)
        M3.duty_ns(throttle_min)
        M4.duty_ns(throttle_min)
        pid_roll.integral = 0
        pid_pitch.integral = 0
        pid_yaw.integral = 0
        print("Throttle is off: ", receiver_data)
        time.sleep(1)
        led.off()

    elif receiver_data[2] > 1010:
        led.on()
        if acro_mode == True:
            ch1_value = int((receiver_data[0] - 1508)/3.0 if receiver_data[0] > 1508 else (receiver_data[0] - 1492)/3.0 if receiver_data[0] < 1492 else 0)
            ch2_value = int((receiver_data[1] - 1508)/3.0 if receiver_data[1] > 1508 else (receiver_data[1] - 1492)/3.0 if receiver_data[1] < 1492 else 0)
            ch3_value = (((receiver_data[2] - 1018) / (1833 - 1018)) * (800 - (0)) + (0))
            ch3_value = max(0, min(ch3_value, 800))
            ch4_value = int((receiver_data[3] - 1508)/3.0 if receiver_data[3] > 1508 else (receiver_data[3] - 1492)/3.0 if receiver_data[3] < 1492 else 0)
        else:
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
        accelerometer_data: list[float] = [(combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x3B, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x3C, 1)) / 4096) -0.01468183,
                                            (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x3D, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x3E, 1)) / 4096) -0.01217782,
                                            (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x3F, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x40, 1)) / 4096) -0.09073687]
        gyro_data: list[float] = [(combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x43, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x44, 1),) / 65.5) - gyro_bias[0],
                                (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x45, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x46, 1),) / 65.5) - gyro_bias[1],
                                (combine_register_values(i2c.readfrom_mem(MPU6050_ADDR, 0x47, 1), i2c.readfrom_mem(MPU6050_ADDR, 0x48, 1),) / 65.5) - gyro_bias[2]]
        
        roll, pitch = calculate_roll_pitch(accelerometer_data, gyro_data, dt, prev_roll, prev_pitch)
        #smoothed_pitch = lp_alpha * prev_pitch + (1 - lp_alpha) * pitch
        #smoothed_roll = lp_alpha * prev_roll + (1 - lp_alpha) * roll
        prev_roll = roll
        prev_pitch = pitch
        #print("Roll:", roll, "Pitch:", pitch)
        #roll_corrected = roll
        #pitch_corrected = pitch

        # Calculating Pid
        if acro_mode == True:
            pid_output_roll = pid_roll.update(ch1_value, gyro_data[0], dt=dt)
            pid_output_pitch =  pid_pitch.update(ch2_value, gyro_data[1], dt=dt)
            pid_output_yaw = pid_yaw.update(ch4_value, gyro_data[2], dt=dt)
        else:
            pid_output_roll = pid_roll.update(ch1_value, roll, dt=dt)
            pid_output_pitch =  pid_pitch.update(ch2_value, pitch, dt=dt)
            pid_output_yaw = pid_yaw.update(ch4_value, gyro_data[2], dt=dt)

        #print("SetPoint: ", ch2_value, "Pitch angle rate: ", gyro_data[1], "Pitch Pid Output:", pid_output_pitch)
        #print("SetPoint: ", ch2_value, "Pitch angle: ", pitch, "Pitch Pid Output:", pid_output_pitch)
        #print(receiver_data)
        #print("Pid Pitch:", pid_output_pitch, "    Pid Roll:", pid_output_roll, "    Pid Yaw:", pid_output_yaw)
        #print("Pid Pitch:", pid_output_pitch, "    Pitch Error:", (ch2_value - gyro_data[1]), "    Pitch Angle:", gyro_data[1], "    Pitch setpoint:", ch2_value)
        #print("Pid Yaw:", pid_output_yaw, "    Yaw Error:", yaw_error, "    Yaw Angle Rate:", gyro_data[2], "    Yaw setpoint:", ch4_value)
        #print("Pid Roll:", pid_output_roll, "    Roll Error:", roll_error, "    Roll Angle:", roll_corrected, "    Roll setpoint:", ch1_value)
        #print("Roll Error: ", roll_error, "Pid roll: ", pid_output_roll,"Pitch error: ", pitch_error, " Pid pitch: ", pid_output_pitch, "ch1value: ", ch1_value, "rollCorrected: ", roll_corrected)
        #print(ch1_value, " ", ch2_value, " ", ch3_value, " ", ch4_value)
        print("X:", gyro_data[0], "Y:", gyro_data[1], "Z:", gyro_data[2])
        motor_mix1 = ch3_value - pid_output_roll + pid_output_pitch + pid_output_yaw # front left, clockwise
        motor_mix1 = max(0, min(motor_mix1, 1000))
        motor_mix2 = ch3_value + pid_output_roll + pid_output_pitch - pid_output_yaw # front right, counter clockwise
        motor_mix2 = max(0, min(motor_mix2, 1000))
        motor_mix3 = ch3_value - pid_output_roll - pid_output_pitch - pid_output_yaw # rear left, counter clockwise
        motor_mix3 = max(0, min(motor_mix3, 1000))
        motor_mix4 = ch3_value + pid_output_roll - pid_output_pitch + pid_output_yaw # rear right, clockwise
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
    time.sleep(0.02)
    #print(dt)
    
    
    