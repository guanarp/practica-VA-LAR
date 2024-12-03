import RPi.GPIO as GPIO
import time
import os
from photo_capture import PhotoCapture
from ultralytics import YOLO
import cv2


GPIO.setmode(GPIO.BOARD)

# Define pins
SENSOR_IN_PIN = 23  # Pin for the sensor input (adjust to your setup)
SENSOR_OUT_PIN = 29
MOTOR_PIN = 36  # Pin for the digital output (adjust to your setup)
FREQ_PIN = 32
SERVOA_PIN = 12 # Use the correct GPIO pin number where your servo is connected
SERVOB_PIN = 33

# Setup de servo
GPIO.setup(SERVOA_PIN, GPIO.OUT)
GPIO.setup(SERVOB_PIN, GPIO.OUT)

pwma = GPIO.PWM(SERVOA_PIN, 50)
pwma.start(0)

pwmb = GPIO.PWM(SERVOB_PIN, 50)
pwmb.start(0)

# Setup GPIO mode
GPIO.setup(SENSOR_IN_PIN, GPIO.IN)  # Sensor input
GPIO.setup(SENSOR_OUT_PIN, GPIO.IN)  # Sensor input
GPIO.setup(MOTOR_PIN, GPIO.OUT)  # Output control
GPIO.setup(FREQ_PIN, GPIO.OUT)  # Output control

# Init motors
GPIO.output(MOTOR_PIN, GPIO.HIGH)  # Turn off output
GPIO.output(FREQ_PIN, GPIO.HIGH)

#model = YOLO("/home/josec/Documents/practica/modelo_Nano.pt")

# Otro modelo mas preciso pero que demora casi 15 segs por frame 
model = YOLO("/home/josec/Documents/practica/modelo_XL.pt")

# Si quieren pueden cambiar su modelo para otro interes, este clasifica naranjas segun calidad

# Si soy la fruta A es el servo de mi izquierda
A_izq = 86  # Angulo correspondientes a los servos
A_med = 50
A_der = 20
B_izq = 115
B_med = 81
B_der = 38

curr_ang_A  = A_med
curr_ang_B = B_med


def run_yolo_inference(model, photos, output_folder="./detections"):
    """
    Run YOLO inference on a list of photos and return formatted results.

    Args:
        model (YOLO): The YOLO model object.
        photos (list): List of photos (numpy arrays) to run inference on.

    Returns:
        list: List of detections in the format [class_id, x_center, y_center, width, height, confidence].
    """
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)  # Create the folder if it doesn't exist
    
    all_detections = []
    for i, photo in enumerate(photos):
        results = model(photo)  # Run YOLO inference
        
        # Save the annotated image
        annotated_image = results[0].plot()  # Create an annotated image
        annotated_image_rgb = cv2.cvtColor(annotated_image, cv2.COLOR_BGR2RGB)
        output_path = os.path.join(output_folder, f"frame_{i + 1}.jpg")
        cv2.imwrite(output_path, annotated_image_rgb)  # Save the image
        

        if results[0].boxes:  # Check if there are detections
            detection = results[0].boxes[0]  # Get the first detection
            # Convert xyxy to x_center, y_center, width, height
            x_min, y_min, x_max, y_max = detection.xyxy.tolist()[0]
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2
            width = x_max - x_min
            height = y_max - y_min

            # Format: [class_id, x_center, y_center, width, height, confidence]
            formatted_detection = [
                int(detection.cls),  # Class ID
                float(x_center),     # x_center
                float(y_center),     # y_center
                float(width),        # width
                float(height),       # height
                float(detection.conf)  # Confidence
            ]
            all_detections.append(formatted_detection)
        else:
            # No detection, append a placeholder or leave empty
            all_detections.append(None)

    return all_detections


def set_servos_angle(angleA, angleB):
	# Calculate duty cycle for the given angle (angle between 0 and 180)
	duty_cycleA = 2.5+ (angleA / 18)
	pwma.ChangeDutyCycle(duty_cycleA)
	duty_cycleB = 2.5+ (angleB / 18)
	pwmb.ChangeDutyCycle(duty_cycleB)
	time.sleep(0.1)  # Allow time for the servo to move to the position
	pwma.ChangeDutyCycle(0)  # Stop sending PWM signal after movement
	pwmb.ChangeDutyCycle(0)

try:
    
    # Create an instance of PhotoCapture and capture photos
    photo_capture = PhotoCapture()
    
    set_servos_angle(A_med, B_med)
    
    print("Inicio del codigo, esperando sensor de entrada")

    while True:
        # Check if SENSOR_IN_PIN is LOW
        if GPIO.input(SENSOR_IN_PIN) == GPIO.LOW:
            print("Reading an object")
            
            # GPIO actions: Turn off FREQ_PIN and MOTOR_PIN
            GPIO.output(FREQ_PIN, GPIO.LOW)
            time.sleep(0.1)  # Wait for 100 ms
            GPIO.output(MOTOR_PIN, GPIO.LOW)
            
            # Esperar a que entre en la recamara
            time.sleep(1.45)
            
            # Capture photos
            photos = photo_capture.capture_photos(count=4, delay=0.4)  # 400ms delay between photos
            print("Photos captured successfully!")
            
            # Esperar a que salga de la recamara y apagar el motor para esperar a que termine de procesar
            time.sleep(0.5)
            GPIO.output(FREQ_PIN, GPIO.HIGH)
            GPIO.output(MOTOR_PIN, GPIO.HIGH)
            
            # Perform YOLO inference on the captured photos
            print("Running YOLO inference...")
            # Run YOLO inference using the function
            detections = run_yolo_inference(model, photos)
            
            print("Frame #              Class X_center Y_center Width Height Confidence")
            # Print the results
            for frame_idx, detection in enumerate(detections):
                if detection:
                    print(f"Frame {frame_idx + 1} Detection: {detection}")
                else:
                    print(f"Frame {frame_idx + 1} Detection: No objects detected.")
                    
            # When inference is done move the fruit
            GPIO.output(FREQ_PIN, GPIO.LOW)
            GPIO.output(MOTOR_PIN, GPIO.LOW)
            
        if GPIO.input(SENSOR_OUT_PIN) == GPIO.LOW:
            print("Fruta saliendo")
            print(detections)
            # Tiempo aprox para que la fruta vaya del sensor al servo
            time.sleep(0.12)
            
            # Verificar si no hubo detectiones
            no_detections = all(d is None for d in detections)
            
            # Verificar si alguna de las detectiones tiene clase 1 o 5
            detected_unwanted_class = any(d[0] in [1, 5] for d in detections if d is not None)
            
            if no_detections or detected_unwanted_class:
                set_servos_angle(A_izq, B_izq)
            
            time.sleep(0.9)
            set_servos_angle(A_med, B_med)
                
            GPIO.output(MOTOR_PIN, GPIO.HIGH)
            GPIO.output(FREQ_PIN, GPIO.HIGH)
            
            print("Codigo listo para recibir otra fruta")
            
        # Short delay to avoid excessive polling
        time.sleep(0.01)



finally:
    print(f"Saved detection images to folder")
    # Stop the camera
    photo_capture.stop_camera()
    GPIO.cleanup()
