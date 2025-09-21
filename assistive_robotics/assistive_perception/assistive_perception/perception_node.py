import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray
from assistive_handover.msg import Handover
import depthai as dai
import blobconverter
import cv2
import numpy as np
from cv_bridge import CvBridge

# Labels for the COCO model
COCO_LABELS = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
    "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite",
    "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
    "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich",
    "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote",
    "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

# List of objects I care about for handover
TARGET_OBJECTS = ["bottle", "cup", "cell phone", "remote"]

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()
        
        self.object_pose_pub = self.create_publisher(Handover, '/handover/object_pose', 10)
        self.hand_pose_pub = self.create_publisher(Handover, '/handover/hand_pose', 10)
        
        self.camera_frame = "oak_d_camera_optical_frame"
        
        self.get_logger().info("Setting up DepthAI pipeline...")
        self.pipeline = dai.Pipeline()
        self.setup_pipeline()
        
        self.device = dai.Device(self.pipeline)
        
        self.obj_q = self.device.getOutputQueue(name="obj_nn", maxSize=4, blocking=False)
        self.hand_q = self.device.getOutputQueue(name="hand_nn", maxSize=4, blocking=False)
        self.spatial_q = self.device.getOutputQueue(name="spatial_data", maxSize=4, blocking=False)
        
        self.timer = self.create_timer(0.03, self.timer_callback)
        self.get_logger().info("Perception node running...")

    def setup_pipeline(self):
        # Color Camera
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(416, 416)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        # Mono Cameras (for depth)
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        depth = self.pipeline.create(dai.node.StereoDepth)
        
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
        
        depth.setDefaultProfilePreset(dai.StereoDepthProperties.ProfileMode.HIGH_DENSITY)
        depth.setLeftRightCheck(True)
        depth.setExtendedDisparity(False)
        depth.setSubpixel(False)
        mono_left.out.link(depth.left)
        mono_right.out.link(depth.right)

        # Spatial Location Calculator
        spatial_calc = self.pipeline.create(dai.node.SpatialLocationCalculator)
        spatial_calc.passthroughDepth.link(depth.depth)
        depth.depth.link(spatial_calc.inputDepth)
        spatial_calc.setWaitForConfigInput(False)
        
        # --- Object Detection (YOLO) ---
        self.get_logger().info("Loading 'yolov4_tiny_coco' from model zoo...")
        obj_nn = self.pipeline.create(dai.node.YoloDetectionNetwork)
        obj_nn.setConfidenceThreshold(0.5)
        obj_nn.setBlobPath(blobconverter.from_zoo(name="yolov4_tiny_coco", shaves=6))
        obj_nn.setNumClasses(80)
        obj_nn.setCoordinateSize(4)
        obj_nn.setAnchors([10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319])
        obj_nn.setAnchorMasks({"side26": [1, 2, 3], "side13": [3, 4, 5]})
        obj_nn.setIouThreshold(0.5)
        cam_rgb.preview.link(obj_nn.input)
        
        # Link object NN to spatial calculator
        obj_nn.passthrough.link(spatial_calc.inputConfig)
        obj_nn.out.link(self.pipeline.create(dai.node.XLinkOut).input)
        self.pipeline.getLastXLinkOut().setStreamName("obj_nn")
        
        # --- Hand Detection ---
        self.get_logger().info("Loading 'hand_landmark_lite' from model zoo...")
        hand_nn = self.pipeline.create(dai.node.NeuralNetwork)
        hand_nn.setBlobPath(blobconverter.from_zoo(name="hand_landmark_lite", shaves=6))
        cam_rgb.preview.link(hand_nn.input) # Use same preview
        
        # Link hand NN to spatial calculator
        hand_nn.passthrough.link(spatial_calc.inputConfig)
        hand_nn.out.link(self.pipeline.create(dai.node.XLinkOut).input)
        self.pipeline.getLastXLinkOut().setStreamName("hand_nn")
        
        # Output for spatial data
        spatial_calc.out.link(self.pipeline.create(dai.node.XLinkOut).input)
        self.pipeline.getLastXLinkOut().setStreamName("spatial_data")

    def timer_callback(self):
        obj_in = self.obj_q.tryGet()
        spatial_in = self.spatial_q.tryGet()

        if obj_in is not None and spatial_in is not None:
            detections = obj_in.detections
            spatial_data = spatial_in.getSpatialLocations()

            for i, detection in enumerate(detections):
                label = COCO_LABELS[detection.label]
                if label in TARGET_OBJECTS:
                    roi = spatial_data[i].config.roi
                    
                    # Get 3D coordinates (in meters, relative to camera)
                    x = spatial_data[i].spatialCoordinates.x / 1000.0
                    y = spatial_data[i].spatialCoordinates.y / 1000.0
                    z = spatial_data[i].spatialCoordinates.z / 1000.0
                    
                    msg = Handover()
                    msg.object_id = label
                    msg.pose.header.frame_id = self.camera_frame
                    msg.pose.header.stamp = self.get_clock().now().to_msg()
                    msg.pose.pose.position.x = x
                    msg.pose.pose.position.y = y
                    msg.pose.pose.position.z = z
                    msg.pose.pose.orientation.w = 1.0 # No orientation data from this model
                    
                    self.object_pose_pub.publish(msg)
                    self.get_logger().info(f"Published '{label}' at [x:{x:.2f}, y:{y:.2f}, z:{z:.2f}]", throttle_duration_sec=2.0)
                    
        # Simplified hand tracking (no spatial data for this simple model)
        hand_in = self.hand_q.tryGet()
        if hand_in is not None:
            # This model is simple. For real use, you'd use a spatial hand tracker.
            # For now, we publish a mock pose to test the logic.
            msg = Handover()
            msg.object_id = "hand"
            msg.pose.header.frame_id = self.camera_frame
            msg.pose.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose.position.x = 0.1 
            msg.pose.pose.position.y = 0.0
            msg.pose.pose.position.z = 0.5
            msg.pose.pose.orientation.w = 1.0
            self.hand_pose_pub.publish(msg)
            self.get_logger().info("Published mock 'hand' pose.", throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.device.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()