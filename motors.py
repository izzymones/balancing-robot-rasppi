from gpiozero import Robot, PWMOutputDevice, DigitalOutputDevice, Button
from time import sleep
import time

ENCODER1_C1 = 23

PWMA = 12
PWMB = 18
AIN1 = 5
AIN2 = 6
BIN1 = 13
BIN2 = 19
STBY = 26

count_per_rotation = 310
wheel_circ = 0.21
distance_per_count = wheel_circ / count_per_rotation



class BRMotors:
	def __init__(self):
		self.motors = Robot((AIN1, AIN2), (BIN1, BIN2))
		self.pwmA = PWMOutputDevice(PWMA)
		self.pwmB = PWMOutputDevice(PWMB)
		self.standby = DigitalOutputDevice(STBY)
		self.encoder1C1 = Button(ENCODER1_C1)
		self.standby.off()
		self.encoder1C1.when_pressed = self.inc_counts
		self.counts = 0
		self.count_inc = 1
		self.last_position = 0
		self.gamma = 0.98
		self.velocity = 0
		self.filtered_velocity = 0
		self.last_time = time.time()

	def inc_counts(self):
		self.counts += self.count_inc
		  
	def position_data(self):
		position = self.counts * distance_per_count

		current_time = time.time()
		dt = current_time - self.last_time if self.last_time else 1e-3
		self.last_time = current_time
		
		self.velocity = (position - self.last_position) / dt
		# self.filtered_velocity = self.gamma * self.filtered_velocity + (1 - self.gamma) * self.velocity

		self.last_position = position

		return position, self.velocity



	def cleanup(self):
		self.motors.stop()
		self.pwmA.close()
		self.pwmB.close()
		self.standby.close()

	def run(self, speed):
		self.standby.on()
		if speed > 0:
			self.count_inc = 1
			self.motors.forward()
		if speed <= 0:
			self.count_inc = -1
			self.motors.backward()
		self.pwmA.value = abs(speed)
		self.pwmB.value = abs(speed)
		
	def stop(self):
		self.motors.stop()


def find_dead_level():
	my_motors = BRMotors()
	for speed in range(0,1000):

		my_motors.run(speed/10)
		sleep(0.01)
		print(speed/10)
		if my_motors.counts > 2:
			break

def main():
	try:
		my_motors = BRMotors()
		my_motors.run(0.25)
		sleep(1)
		my_motors.run(-0.25)
		sleep(1)
		print(my_motors.position_data())
	finally:
		my_motors.cleanup()

if __name__ == "__main__":
	main()

