import rclpy
from rclpy.node import Node
import depthai as dai
import cv2
import numpy as np

class ParkingController(Node):
    def __init__(self):
        super().__init__('parking_controller')
        self.get_logger().info('Parking Controller (Sanity Check Mode) Started')
        self.run_depthai()

    def run_depthai(self):
        # --- 1. Create Pipeline ---
        pipeline = dai.Pipeline()

        # Define Color Camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        
        # OPTIMIZATION: Set preview to 640x480 (VGA) for high-speed detection
        # This makes the AprilTag algorithm run much faster than on 1080p
        cam_rgb.setPreviewSize(640, 480)
        
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(30)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        
        # Force Auto-Focus
        cam_rgb.initialControl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)

        # Define AprilTag Node
        april_tag = pipeline.create(dai.node.AprilTag)
        
        # Config: Use 36h11 Family
        april_config = april_tag.initialConfig.get()
        april_config.family = dai.AprilTagConfig.Family.TAG_36H11
        april_tag.initialConfig.set(april_config)

        # CRITICAL FIX: Link the smaller 'preview' to AprilTag input (not high-res video)
        cam_rgb.preview.link(april_tag.inputImage) 

        # Create XLink Outputs
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input) # Stream the 640x480 preview

        xout_april = pipeline.create(dai.node.XLinkOut)
        xout_april.setStreamName("april")
        april_tag.out.link(xout_april.input)

        # --- 2. Connect to Device ---
        self.get_logger().info("Connecting to OAK-D (High Speed Mode)...")
        with dai.Device(pipeline) as device:
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_april = device.getOutputQueue(name="april", maxSize=4, blocking=False)

            self.get_logger().info("OAK-D Running at 640x480. Press 'q' to quit.")

            while rclpy.ok():
                in_rgb = q_rgb.tryGet()
                in_april = q_april.tryGet()

                if in_rgb is not None:
                    frame = in_rgb.getCvFrame()

                    if in_april is not None:
                        for detection in in_april.aprilTags:
                            # No interpolation needed if we display the raw 640x480 preview!
                            # The detection coordinates match the frame size exactly now.
                            xTL = int(detection.topLeft.x)
                            yTL = int(detection.topLeft.y)
                            xBR = int(detection.bottomRight.x)
                            yBR = int(detection.bottomRight.y)
                            
                            center_x = int(xTL + (xBR - xTL) / 2)
                            center_y = int(yTL + (yBR - yTL) / 2)

                            # Visuals
                            cv2.rectangle(frame, (xTL, yTL), (xBR, yBR), (0, 255, 0), 2)
                            cv2.putText(frame, f"ID: {detection.id}", (xTL, yTL - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            
                            self.get_logger().info(f"Tag Found! ID: {detection.id} | Center: ({center_x}, {center_y})")

                    cv2.imshow("OAK-D AprilTag High-Speed", frame)

                if cv2.waitKey(1) == ord('q'):
                    break

            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = ParkingController()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
