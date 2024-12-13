from machine import Pin, PWM
import time

# Pin definitions
TRIG_PIN = 13  # Trigger pin for HC-SR04
ECHO_PIN = 14  # Echo pin for HC-SR04
SERVO1_PIN = 15  # Servo 1 pin
SERVO2_PIN = 16  # Servo 2 pin

# Threshold distance in cm to deploy landing gear
THRESHOLD = 30

# Servo angles in duty cycles (for a typical SG90 servo)
ANGLE1 = 0.5  # 0 degrees (adjust for your servos)
ANGLE2 = 2.5  # 160 degrees (adjust for your servos)

# Setup ultrasonic sensor pins
trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)

# Setup servos as PWM outputs
servo1 = PWM(Pin(SERVO1_PIN))
servo2 = PWM(Pin(SERVO2_PIN))
for servo in [servo1, servo2]:
    servo.freq(50)  # Set frequency to 50Hz for servo motors

# Function to move servo to a specific angle
def move_servo(servo, angle):
    duty = 0.5 + (angle / 180) * 2  # Convert angle to duty cycle
    servo.duty_u16(int(duty * 65536 / 20))  # Convert to 16-bit duty cycle

# Function to read distance from HC-SR04
def get_distance():
    # Send trigger pulse
    trig.low()
    time.sleep_us(2)
    trig.high()
    time.sleep_us(10)
    trig.low()

    # Wait for echo to start
    while echo.value() == 0:
        pass
    start = time.ticks_us()

    # Wait for echo to stop
    while echo.value() == 1:
        pass
    end = time.ticks_us()

    # Calculate distance in cm
    duration = time.ticks_diff(end, start)
    distance = (duration / 2) / 29.1  # Speed of sound in cm/us
    return distance

# Initialize servos to starting position
move_servo(servo1, 0)
move_servo(servo2, 0)

# Main loop
try:
    while True:
        distance = get_distance()
        print(f"Distance: {distance:.2f} cm")
        
        if distance < THRESHOLD:
            # Deploy landing gear
            print("Deploying landing gear")
            move_servo(servo1, 180)
            move_servo(servo2, 180)
        else:
            # Retract landing gear
            print("Retracting landing gear")
            move_servo(servo1, 0)
            move_servo(servo2, 0)

        time.sleep(0.5)  # Wait before next measurement

except KeyboardInterrupt:
    print("Program stopped")
    servo1.deinit()
    servo2.deinit()
