from picamera2 import Picamera2
from time import sleep
import numpy as np

class PhotoCapture:
    def __init__(self):
        # Initialize Picamera2 with custom configuration
        self.picam2 = Picamera2()
        self.camera_config = self.picam2.create_still_configuration(
            main={"size": (1280, 720), "format": "BGR888"},  # Set resolution and format
            controls={"FrameRate": 24},  # Set frame rate
            display="main"  # Enable display
        )
        self.picam2.configure(self.camera_config)
        self.picam2.start()

    def capture_photos(self, count=3, delay=0.5):
        """
        Captures a specified number of photos with a delay between each.
        
        Args:
        - count (int): Number of photos to capture.
        - delay (float): Delay between captures in seconds.

        Returns:
        - List of captured photos as numpy arrays.
        """
        photos = []
        for _ in range(count):
            # Capture photo and append to list
            photo = self.picam2.capture_array()
            photos.append(photo)
            sleep(delay)  # Wait before capturing the next photo
        return photos

    def stop_camera(self):
        """Stops the Picamera2 instance."""
        self.picam2.stop()

# Usage Example
if __name__ == "__main__":
    try:
        # Create an instance of PhotoCapture
        photo_capture = PhotoCapture()

        # Capture 3 photos spaced 500ms apart
        photos = photo_capture.capture_photos(count=3, delay=0.5)
        print(f"Captured {len(photos)} photos.")

        # Process or save photos here if needed
        for i, photo in enumerate(photos):
            print(f"Photo {i + 1}: {photo.shape}")  # Example: print the resolution
    finally:
        # Clean up
        photo_capture.stop_camera()
