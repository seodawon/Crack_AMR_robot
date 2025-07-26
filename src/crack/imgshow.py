import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import os

# Global shared variables
latest_rgb_compressed = None
shutdown_requested = False

image_count = 1  # Counter for saved images

display_mode = 1  # default: preview + depth

class CameraSubscriber(Node):
    def __init__(self, save_directory=None, file_prefix=None):
        super().__init__('camera_subscriber')
        self.save_directory = save_directory
        self.file_prefix = file_prefix
        self.rgb_compressed_sub = self.create_subscription(
            CompressedImage, '/robot7/oakd/rgb/image_raw/compressed', self.rgb_compressed_callback, 10)

    def rgb_compressed_callback(self, msg):
        global latest_rgb_compressed, image_count
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            decoded = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            latest_rgb_compressed = cv2.resize(decoded, (640, 480))
            self.get_logger().info('Received image')

            # Display the image using OpenCV
            cv2.imshow('Received Image', latest_rgb_compressed)
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                self.get_logger().info("Shutdown requested by user.")
                rclpy.shutdown()

            elif key == ord('c') and self.save_directory and self.file_prefix:
                image_count += 1
                image_path = os.path.join(self.save_directory, f"{self.file_prefix}_img_{image_count}.jpg")
                cv2.imwrite(image_path, latest_rgb_compressed)
                self.get_logger().info(f"Image saved: {image_path}")

        except Exception as e:
            self.get_logger().error(f"RGB compressed conversion failed: {e}")


def main(args=None):
    rclpy.init(args=args)

    # Example: Set directory and prefix here
    save_directory = '/home/park/Downloads/slam3'
    file_prefix = 'test'

    camera_subscriber = CameraSubscriber(save_directory, file_prefix)

    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()  # Close OpenCV windows
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
