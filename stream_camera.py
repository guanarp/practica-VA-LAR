import cv2
from picamera2 import Picamera2

def stream_camera():
    """
    Captures and displays a live stream from the camera using Picamera2 and OpenCV.
    """
    # Initialize Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (1280, 720), "format": "BGR888"})
    picam2.configure(config)
    picam2.start()

    print("Press 'q' to quit the camera stream.")
    
    try:
        while True:
            # Capture a frame
            frame = picam2.capture_array()
            
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Display the frame
            cv2.imshow("Camera Stream", frame_bgr)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # Clean up resources
        picam2.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    stream_camera()
