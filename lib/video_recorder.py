import cv2
import numpy as np 

class VideoRecorder:
    def __init__(self, filename, codec='mp4v', fps=20.0, frame_size=(640, 480)):
        """
        Initialize the VideoRecorder.

        :param filename: The name of the output video file.
        :param codec: The codec to be used (default is 'mp4v' for MP4 format).
        :param fps: The frame rate of the output video (default is 20.0).
        :param frame_size: The size of the video frames (width, height) (default is (640, 480)).
        """
        self.filename = filename
        self.codec = cv2.VideoWriter_fourcc(*codec)
        self.fps = fps
        self.frame_size = frame_size
        self.writer = None

        self.start()

    def start(self):
        """
        Start the video recording.
        """
        self.writer = cv2.VideoWriter(self.filename, self.codec, self.fps, self.frame_size)

    def write_frame(self, frame):
        """
        Write a frame to the video.

        :param frame: The frame to be written (numpy array).
        """
        if self.writer is not None:
            self.writer.write(frame)
        else:
            pass

    def release(self):
        """
        Release the VideoWriter and save the video file.
        """
        if self.writer is not None:
            self.writer.release()
            self.writer = None
            print(f"Video saved as {self.filename}")
        else:
            pass

# Example usage
if __name__ == "__main__":
    # Create an instance of VideoRecorder
    recorder = VideoRecorder(filename='output.mp4', fps=30.0, frame_size=(800, 600))
    
    # Start recording
    recorder.start()
    
    # Generate and write frames
    for frame_number in range(100):
        # Create a blank frame
        frame = np.zeros((600, 800, 3), dtype=np.uint8)
        
        # Draw a rectangle that moves across the frame
        x = int(frame_number * 800 / 100)
        cv2.rectangle(frame, (x, 200), (x + 50, 250), (0, 255, 0), -1)
        
        # Write the frame to the video
        recorder.write_frame(frame)
    
    # Release the VideoWriter and save the video
    recorder.release()
