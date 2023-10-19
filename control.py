import rospy
import moveit_commander
import geometry_msgs
from Transformation import Transform
import actionlib
import franka_gripper.msg
import actionlib
import requests
from std_msgs.msg import Float32MultiArray

bins = {
    1: [0.236148826955,-0.308433911665, 0.2129949191],
    2: [0.15, 0.48, 0.20]
}

labels = {
    0: "anorthosite",
    1: "basalt",
    2: "metal",
    3: "regolith"
}

class Panda:
    def __init__(self):
        # Initialize the node
        # rospy.init_node('move_panda_node', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # Initialize the planning group
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        
        self.addTable()

        # Set the planning time
        self.group.set_planning_time(5.0)

        # Set the reference frame
        self.group.set_pose_reference_frame("panda_link0") # Sending pose with respect to base frame

        # Set the velocity scaling
        self.group.set_max_velocity_scaling_factor(0.1)
        # Set the acceleration scaling
        self.group.set_max_acceleration_scaling_factor(0.1)

        # Subscriber to /coordinates
        self.coordinates_subcriber = rospy.Subscriber('coordinates', Float32MultiArray, self.callback)
        self.coordinates = []

        self.positions = {
            1: [1.6085718848077872, 1.1893688026023979, -1.1578405069772622, -1.2433873730541787, 0.32506408105953705, 1.3915502173900605, 1.2802830292667389],
            2: [1.677193905098436, 0.935329898052105, -1.236179748802854, -1.2934699724598935, 0.2488373336561151, 1.3600093161058306, 1.2278366222104506],
            3: [1.8428348618070094, 0.5372377344934564, -1.473488105793616, -1.4416147330936633, 0.11911833076635508, 1.355531926870346, 1.1173401238206444]
        }
    
    def callback(self, msg):
        self.coordinates = msg.data

    def move_pose(self, pos=None, angle=None):   # # Create an orientation constraint

        pose = self.group.get_current_pose()
        joint_angles = self.group.get_current_joint_values()

        # print("Current pose: ", self.group.get_current_pose())

        # print("Joint angles: ", joint_angles)
        
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        self.group.set_pose_target(pose)
        self.group.go(wait=True)

       

    def addTable(self):
        import time; time.sleep(2)
        table = geometry_msgs.msg.PoseStamped()
        table.header.frame_id = "world"
        table.pose.orientation.w = 1.0
        table.pose.position.z -= 0.15
        table_name = "table"
    
        self.scene.add_box(table_name, table, size=(5, 5, 0.1))
        print("added table..")


    def move_robot(self, pose):
        joint_goal = self.group.get_current_joint_values()
        
        joint_goal[0] = pose[0]
        joint_goal[1] = pose[1]
        joint_goal[2] = pose[2]
        joint_goal[3] = pose[3]
        joint_goal[4] = pose[4]
        joint_goal[5] = pose[5]
        joint_goal[6] = pose[6]

        self.group.go(joint_goal, wait=True)

        # Clear the pose target
        self.group.clear_pose_targets()

    def reset(self, camera_mode):
        home_pose = {1 : [0.0006584152512235992, -0.189610766995982, 0.04070987412908618, 
                        -1.7441984363054845, 0.012076929801793831, 1.6136773423439779, 
                        0.8541646518984198],
                    2 : [0.15317611124222738, -0.2321941326354679, -0.08839137087579359, 
                        -2.210608805639227, -0.021735957427656678, 1.993725836197535, 
                        0.8536625544818073]}
        if camera_mode == True:
            self.move_robot(home_pose[1])
        else:
            self.move_robot(home_pose[1])
    
    def grasp(self, width=0, e_inner=0.1, e_outer=0.1, speed=0.1, force=1):
        """
        Wrapper around the franka_gripper/grasp action.
        http://docs.ros.org/kinetic/api/franka_gripper/html/action/Grasp.html
        :param width: Width (m) to grip
        :param e_inner: epsilon inner
        :param e_outer: epsilon outer
        :param speed: Move velocity (m/s)
        :param force: Force to apply (N)
        :return: Bool success
        """
        client = actionlib.SimpleActionClient('franka_gripper/grasp', franka_gripper.msg.GraspAction)
        client.wait_for_server()
        client.send_goal(
            franka_gripper.msg.GraspGoal(
                width,
                franka_gripper.msg.GraspEpsilon(e_inner, e_outer),
                speed,
                force
            )
        )
        return client.wait_for_result()

def fetch_values():
    response = requests.get("http://127.0.0.1:5000/get_coords")
    if response.status_code == 200:
        return response.json()['coords']
    else:
        print("Error:", response.status_code)
        return []
    
def pickup_and_drop_item(robot, pose, bin_no):
    print("Resetting 1")
    robot.reset(infrared_mode)
    print("Move to above object")
    robot.move_pose([pose[0][0], pose[1][0], 0.3])
    print("Move down to object")
    robot.move_pose([pose[0][0], pose[1][0], 0.1])
    print("Grasp object")
    result = robot.grasp( width=0.00, e_inner=1, e_outer=1, speed=0.1, force=2)
    robot.reset(infrared_mode)
    print("Move to bin")
    robot.move_pose(bins[bin_no])
    print("Drop object")
    result = robot.grasp(width=0.1, e_inner=1, e_outer=1, speed=0.1, force=2)




if __name__ == "__main__":
    infrared_mode = False

    rospy.init_node("robot_arm")
    robot = Panda()

    robot.reset(infrared_mode)
    result = robot.grasp( width=0.1, e_inner=1, e_outer=1, speed=0.1, force=1)
    transform = Transform(infrared=infrared_mode)

    # Rotate the bot to angle
    #robot.move_robot(robot.positions[1])
 
    while True:
        if raw_input('Continue?') == '0':
            break

        #print("Number of objects found: ", len(robot.coordinates))
        print("Next rock pixel centre, depth:", robot.coordinates)

        if robot.coordinates:
            width, height, depth = robot.coordinates
            print(width, height, depth)
            pose = transform.get_pos([width, height, depth])
            bin_no = 2
            pickup_and_drop_item(robot, pose, bin_no)
            print("Reseting to camera position")
            robot.reset(infrared_mode)

    print("All done!")
    
    




    



