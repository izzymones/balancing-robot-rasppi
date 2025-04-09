from mpu6050 import mpu6050 
import time
import math
import json

class ImuSensor:

    # create a gyro sensor and calibrate it
    def __init__(self):
        self.sensor = mpu6050(0x68)
        self.calibrate_gyro(500)

        self.ax_raw = 0.0
        self.ay_raw = 0.0
        self.az_raw = 0.0

        self.g_raw = 0.0
        self.g_previous = 0.0
        self.g_high_pass = 0.0

        self.ax_low_pass = 0.0
        self.ay_low_pass = 0.0
        self.az_low_pass = 0.0

        self.angle = 0.0
        self.angular_velocity = 0

        cutoff_frequency = 5
        tau = 1/(2*math.pi*cutoff_frequency)
        self.alpha = tau / (tau*self.dT)
        
        self.dT = 0.005
        self.beta = 0.98

    def calibrate_gyro(self, samples):
        cum_bias_x = 0
        for _ in range(samples):
            data = self.sensor.get_gyro_data()
            cum_bias_x += data['x']
            time.sleep(0.002)
        self.bias_x = cum_bias_x / samples
        print("gyro x bias: ", self.bias_x)

    def low_pass(self):
        self.ax_low_pass = (1 - self.alpha) * self.ax_raw + self.alpha * self.ax_low_pass
        self.ay_low_pass = (1 - self.alpha) * self.ay_raw + self.alpha * self.ay_low_pass
        self.az_low_pass = (1 - self.alpha) * self.az_raw + self.alpha * self.az_low_pass
        return self.ax_low_pass, self.ay_low_pass, self.az_low_pass

    
    def high_pass(self):
        self.g_high_pass = (1 - self.alpha) * self.g_high_pass + (1 - self.alpha) * (self.g_raw - self.g_previous)

        self.g_previous = self.g_raw

        return self.g_high_pass

    def complementary_filter(self):
        self.angle_previous = self.angle
        self.low_pass()
        self.high_pass()
        self.angle = self.angle - self.g_high_pass * self.dT
        accelerometer_angle = math.atan2(self.ay_low_pass, math.sqrt(self.ax_low_pass**2 + self.az_low_pass**2))
        accelerometer_angle = accelerometer_angle * 180 / math.pi  
        self.angle = (1 - self.beta) * self.angle + (self.beta)*(accelerometer_angle)
        self.angular_velocity = (self.angle - self.angle_previous) / self.dT
        return self.angle * math.pi / 180, self.angular_velocity * math.pi / 180

def output_data():
    imu = ImuSensor()  
    counter = 0  
    run_data = []
    for i in range(500):
        angle, angular_velocity = imu.complementary_filter()
        run_data.append([angle, angular_velocity])
        time.sleep(0.01)
    
    # Serializing json
    json_object = json.dumps(run_data, indent=4)
 
    # Writing to sample.json
    with open("gyro.json", "w") as outfile:
        outfile.write(json_object)

