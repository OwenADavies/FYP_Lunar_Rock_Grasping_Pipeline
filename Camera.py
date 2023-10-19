import cv2
import pyrealsense2 as rs
import numpy as np
import time
from yolov8 import YOLOv8
import rospy
from std_msgs.msg import Float32MultiArray

class Camera:
    def __init__(self, infrared=False):
      # Configure the RealSense camera
      self.pipeline = rs.pipeline()
      self.config = rs.config()

      # Init ros node
      rospy.init_node("FYP")

      # Create a publisher
      self.coordinates_publisher = rospy.Publisher('coordinates', Float32MultiArray, queue_size=10)
      self.rate = rospy.Rate(10)


      #self.model_path = "code/models/ir_v2.onnx"
      self.model_path_rgb = "code/models/RGB_weights.onnx"
      self.yolov8_detector_rgb = YOLOv8(self.model_path_rgb, conf_thres=0.5, iou_thres=0.2)
      self.model_path_ir = "code/models/IR_weights.onnx"
      self.yolov8_detector_ir = YOLOv8(self.model_path_ir, conf_thres=0.5, iou_thres=0.2)


      self.infrared = infrared

      # Enable Infrared and Depth stream
      self.config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 15)
      #self.config.enable_stream(rs.stream.infrared, 2, 1280, 720, rs.format.y8, 15)
      self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)
      self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

      # Start the RealSense pipeline
      self.profile = self.pipeline.start(self.config)

      ir_sensor = self.profile.get_device().query_sensors()[0]
      ir_sensor.set_option(rs.option.enable_auto_exposure, True) # Was set to True
      #ir_sensor.set_option(rs.option.exposure, 165000.0) # Was set at 8500, range is 1-165000

      colour_sensor = self.profile.get_device().query_sensors()[1]
      colour_sensor.set_option(rs.option.enable_auto_exposure, True) # Was set to True
      #colour_sensor.set_option(rs.option.exposure, 10000.0) # Was set atexposure 166, range is 1-10000
      #colour_sensor.set_option(rs.option.gain, 128) # Default 64, range 0-128

      self.depth_sensor = self.profile.get_device().first_depth_sensor()
      # self.depth_sensor.set_option(rs.option.emitter_enabled, 1)
      # self.depth_sensor.set_option(rs.option.laser_power, 150)

      self.depth_sensor.set_option(rs.option.emitter_enabled, 1.0)
      self.depth_sensor.set_option(rs.option.laser_power, 150.0)

      # Defining vaiables
      self.real_x, self.real_y, self.real_z = 0, 0, 0
      self.center_x, self.center_y, self.depth = 0, 0, 0

    def get_frames(self):
      frames = self.pipeline.wait_for_frames()

      align = rs.align(rs.stream.infrared)
      frames = align.process(frames)

      depth_frame = frames.get_depth_frame()
      ir_frame = frames.get_infrared_frame()
      rgb_frame = frames.get_color_frame()

      while not depth_frame or not ir_frame or not rgb_frame:
        frames = self.pipeline.wait_for_frames()

        align = rs.align(rs.stream.infrared)
        frames = align.process(frames)

        depth_frame = frames.get_depth_frame()
        ir_frame = frames.get_infrared_frame()
        rgb_frame = frames.get_color_frame()

      return depth_frame, rgb_frame, ir_frame

    def get_coordinates(self):

      center_list = []

      #self.depth_sensor.set_option(rs.option.emitter_enabled, 1.0)
      self.depth_sensor.set_option(rs.option.laser_power, 150.0)

      time.sleep(1)

      depth_frame, _, _ = self.get_frames()

      #self.depth_sensor.set_option(rs.option.emitter_enabled, 0.0)
      self.depth_sensor.set_option(rs.option.laser_power, 0.0)

      time.sleep(1)

      _, rgb_frame, ir_frame = self.get_frames()


      rgb_img = np.asanyarray(rgb_frame.get_data())
      rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)
      ir_img  = np.asanyarray(ir_frame.get_data())
      ir_img = cv2.cvtColor(ir_img, cv2.COLOR_GRAY2BGR)


      ir_img1 = ir_img.copy()
      boxes, scores, class_ids = self.yolov8_detector_ir(ir_img)

      ir_img1 = self.yolov8_detector_ir.draw_detections(ir_img)
      for class_id, box, score in zip(class_ids, boxes, scores):
         x1, y1, x2, y2 = box.astype(int)
         #cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
         self.center_x = int((x1 + x2) / 2)
         self.center_y = int((y1 + y2) / 2)
         cv2.circle(ir_img1, (self.center_x, self.center_y), 10, (255,0,0), -1)


      rgb_img1 = rgb_img.copy()
      boxes, scores, class_ids = self.yolov8_detector_rgb(rgb_img)

      rgb_img1 = self.yolov8_detector_rgb.draw_detections(rgb_img)
      for class_id, box, score in zip(class_ids, boxes, scores):
         x1, y1, x2, y2 = box.astype(int)
         #cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
         self.center_x = int((x1 + x2) / 2)
         self.center_y = int((y1 + y2) / 2)
         cv2.circle(rgb_img1, (self.center_x, self.center_y), 10, (255,0,0), -1)
         (img_h, img_w) = rgb_img1.shape[:2]
         #cv2.circle(img1, (img_w//2, img_h//2), 10, (0,255,0), -1)

         self.depth = depth_frame.get_distance(self.center_x, self.center_y)
         print(self.center_x, self.center_y)

         depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
         self.real_x, self.real_y, self.real_z = rs.rs2_deproject_pixel_to_point(depth_intrin, [self.center_x, self.center_y], self.depth)
        #  center_list.append([real_x, real_y, real_z])

      # publishing data to ros /coordinates topic
      message_to_pub = Float32MultiArray()
      message_to_pub.data = [self.center_x, self.center_y, self.depth]
      self.coordinates_publisher.publish(message_to_pub)

      self.rate.sleep()

      # Press 'q' to exit the loop and close the window
      cv2.imshow("Object detect IR", ir_img1)

      cv2.imshow("Original IR", ir_img)

      cv2.imshow("Object detect RGB", rgb_img1)

      cv2.imshow("Original RGB", rgb_img)

      cv2.waitKey(1)
      
    def _stop(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()  


if __name__ == '__main__':
  try:
      camera = Camera(infrared=False)
      while not rospy.is_shutdown():
         camera.get_coordinates()

  except rospy.ROSInternalException:
     pass