import RPi.GPIO as GPIO
import time

# GPIO Pin Definitions
SENSOR_IN_PIN = 23  # Sensor input pin
SENSOR_OUT_PIN = 29  # Sensor output pin (optional for testing)
MOTOR_PIN = 36  # Motor control pin
FREQ_PIN = 32  # Frequency control pin
SERVO_A_PIN = 33  # Servo motor A pin
SERVO_B_PIN = 35  # Servo motor B pinn

# Servo angle definitions
A_izq = 86  # Left angle for servo A
A_med = 50  # Middle angle for servo A
A_der = 20  # Right angle for servo A
B_izq = 115  # Left angle for servo B
B_med = 81  # Middle angle for servo B
B_der = 38  # Right angle for servo B

# Current servo angles
curr_ang_A = A_med
curr_ang_B = B_med

GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering

# Set up input pins
GPIO.setup(SENSOR_IN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Pull-up for stable input
GPIO.setup(SENSOR_OUT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Optional pull-up

# Set up output pins
GPIO.setup(MOTOR_PIN, GPIO.OUT)
GPIO.setup(FREQ_PIN, GPIO.OUT)

# Initialize output pins
GPIO.output(MOTOR_PIN, GPIO.HIGH)  # Turn off motor initially
GPIO.output(FREQ_PIN, GPIO.HIGH)   # Turn off frequency control initially

# Set up servo pins as output
GPIO.setup(SERVO_A_PIN, GPIO.OUT)
GPIO.setup(SERVO_B_PIN, GPIO.OUT)

# Initialize PWM for servos (50 Hz for typical servos)
pwma = GPIO.PWM(SERVO_A_PIN, 50)
pwmb = GPIO.PWM(SERVO_B_PIN, 50)
pwma.start(0)  # Start with duty cycle 0
pwmb.start(0)

def set_servos_angle(angleA, angleB):
    """
    Sets the angles of the servo motors.

    Args:
        angleA (int): Angle for servo A (0 to 180).
        angleB (int): Angle for servo B (0 to 180).
    """
    duty_cycleA = 2.5 + (angleA / 18)
    duty_cycleB = 2.5 + (angleB / 18)
    pwma.ChangeDutyCycle(duty_cycleA)
    pwmb.ChangeDutyCycle(duty_cycleB)
    time.sleep(0.5)  # Allow time for the servo to move
    pwma.ChangeDutyCycle(0)  # Stop sending PWM signal
    pwmb.ChangeDutyCycle(0)

set_servos_angle(A_med, B_med)

def test_gpio():
    """
    Tests GPIO inputs and outputs.
    """
    print("Starting GPIO test. Press Ctrl+C to exit.")
    print()
    
    # Initialize previous states for comparison
    prev_sensor_in_state = GPIO.input(SENSOR_IN_PIN)
    prev_sensor_out_state = GPIO.input(SENSOR_OUT_PIN)
    try:
        while True:
            # Read input state
            sensor_in_state = GPIO.input(SENSOR_IN_PIN)
            sensor_out_state = GPIO.input(SENSOR_OUT_PIN)

            # Check if SENSOR_IN_PIN state has changed
            if sensor_in_state != prev_sensor_in_state:
                print(f"Sensor de entrada: {'LOW (Activado)' if sensor_in_state == GPIO.LOW else 'HIGH (Inactivo)'}")
                prev_sensor_in_state = sensor_in_state  # Update previous state

            # Check if SENSOR_OUT_PIN state has changed
            if sensor_out_state != prev_sensor_out_state:
                print(f"Sensor de salida: {'LOW (Activado)' if sensor_out_state == GPIO.LOW else 'HIGH (Inactivo)'}")
                prev_sensor_out_state = sensor_out_state  # Update previous state

            # Toggle motor and frequency outputs based on SENSOR_IN_PIN
            if sensor_in_state == GPIO.LOW:  # Sensor triggered
                print("Entrada activa, encendiendo motor por 2 segs")
                GPIO.output(MOTOR_PIN, GPIO.LOW)  # Turn motor ON
                GPIO.output(FREQ_PIN, GPIO.LOW)  # Turn frequency control ON
                time.sleep(2)
                GPIO.output(MOTOR_PIN, GPIO.HIGH)  # Turn motor ON
                GPIO.output(FREQ_PIN, GPIO.HIGH)  # Turn frequency control ON
                print("Apagando motor")
                print()
            
            if sensor_out_state == GPIO.LOW:
                print("Salida activa, moviendo servomotores")
                print()
                set_servos_angle(A_izq, B_izq)
                time.sleep(1)
                set_servos_angle(A_med, B_med)
            

            time.sleep(0.5)  # Adjust delay as needed

    except KeyboardInterrupt:
        print("\nExiting GPIO test.")

    finally:
        # Cleanup GPIO settings
        GPIO.cleanup()
        print("GPIO cleaned up.")

if __name__ == "__main__":
    test_gpio()
