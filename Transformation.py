import tf
import rospy
import numpy as np
import time

# # RGB Camera intrinsics
# fx_rgb = 389.691
# fy_rgb = 389.691
# cx_rgb = 321.702
# cy_rgb = 235.971
K_rgb = [605.9451904296875, 0.0, 321.40673828125, 0.0, 605.49072265625, 249.2628173828125, 0.0, 0.0, 1.0]
fx_rgb = K_rgb[0]
cx_rgb = K_rgb[2]
fy_rgb = K_rgb[4]
cy_rgb = K_rgb[5]

# IR Camera intrinsics
K_ir = [389.691162109375, 0.0, 321.70245361328125, 0.0, 389.691162109375, 235.97076416015625, 0.0, 0.0, 1.0]
fx_ir = K_ir[0]
cx_ir = K_ir[2]
fy_ir = K_ir[4]
cy_ir = K_ir[5]

class Transform:
  def __init__(self, infrared=False):
    # rospy.init_node('Transform_listen', anonymous=True)

    self.listener = tf.TransformListener()
    if infrared:
      self.infrared_mode = True
    else:
      self.infrared_mode = False

    # self.camera_info = rospy.wait_for_message("/camera/depth/camera_info", CameraInfo)

    # self.fx = self.camera_info.K[0]
    # self.fy = self.camera_info.K[5]
    # self.cx = self.camera_info.K[2]
    # self.cy = self.camera_info.K[6]
    
    # print(self.fx, self.fy, self.cx, self.cy)
    
    self.rate = rospy.Rate(10.0)

  


  def get_pos(self, camera_coordinates):
    try:
      if camera_coordinates:
        if self.infrared_mode:
          # For infrared camera
          v, u, z = camera_coordinates
          #print(u, v, z)
          x = (u - cx_ir) * z  / fx_ir
          y = (v - cy_ir) * z  / fy_ir
          x = x
          y = y
        else:
          u, v, z = camera_coordinates
          x = (u - cx_rgb) * z / fx_rgb
          y = (v - cy_rgb) * z / fy_rgb

          x = x - 0.05
          y = y + 0.015
        # else:
        #   # For colour camera
        #   x, y, z = camera_coordinates
        #   x_offset = 0.115
        #   y_offset = 0.01
        
        print("Rock position on ground:", (x, y, z))
        
        P_camera = np.array([-x, -y, z, 1]).reshape(4,1) 

        time.sleep(1)

        # infra_2_color_trans = [-0.000, 0.015, 0.000]
        # infra_2_color_rot = [0.009, -0.001, -0.000, 1.000]

        # infra_2_color_trans2 = [-0.015, -0.000, -0.000]
        # infra_2_color_rot2 = [0.505, -0.496, 0.504, 0.495]

        # R = tf.transformations.quaternion_matrix(infra_2_color_rot)
        # T = R.copy()
        # T[0:3, 3] = infra_2_color_trans

        # P_rgb_to_ir = np.dot(T, P_camera)


        (trans, rot) = self.listener.lookupTransform('panda_link0', 'panda_EE', rospy.Time(0))

        # (t, r) = self.listener.lookupTransform("camera_color_optical_frame", "panada_link8", rospy.Time(0))
        # print(t,r)        
        R = tf.transformations.quaternion_matrix(rot)
        T = R.copy()
        T[0:3, 3] = trans

        if self.infrared_mode:
          # cam_trans = [0.037, -0.017, -0.040]
          # cam_rot = [0.707, -0.000, 0.707, -0.000]
          # cam_trans = [0.040, -0.018, -0.037]
          # cam_rot = [0.000, 0.000, 0.707, 0.707]

          cam_rot = [-0.000, -0.001, -0.714, 0.701]
          cam_trans = [0.033, 0.055, 0.037]
        else:
          cam_rot = [-0.000, -0.001, -0.714, 0.701]
          cam_trans = [0.033, 0.040, 0.037]

        # cam_rot = [-0.000, -0.001, -0.714, 0.701]
        # cam_trans = [0.033, 0.040, 0.037]



       
        R_cam = tf.transformations.quaternion_matrix(cam_rot)
        T_end_to_camera = R_cam.copy()
        T_end_to_camera[0:3, 3] = cam_trans

        T_camera_to_base = np.dot(T, T_end_to_camera)

        waypoints = np.dot(T_camera_to_base, P_camera)
        print(waypoints)

        return waypoints
      
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as error:
      print("An exception has occured: ", error)
      self.rate.sleep()
      return
    
    else: 
      print("Nothing received.")