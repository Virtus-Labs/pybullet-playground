import pybullet as p
import pybullet_data
import time
import threading
import tkinter as tk
from tkinter import filedialog
import os
import shutil
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Create a minimal node for publishing
class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('pybullet_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

class RobotSimulation:
    def __init__(self):
        # Initialize ROS2
        rclpy.init()
        
        #  Create minimal publisher node
        self.ros_node = MinimalPublisher()

        # Initialize PyBullet
        self.physicsClient = p.connect(p.GUI,options="--disable_grid=1")
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
        p.setGravity(0, 0, -9.8)
        
        # Configure debug visualization
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.resetDebugVisualizerCamera(
            cameraDistance=3.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )

        self.create_flat_ground()
        self.create_obstacles()
        # Initialize robot state
        self.robot = None
        self.wheel_joints = {} 
        
        # Initialize wheel velocities
        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0
        
        # Create temp directory for URDF files
        if not os.path.exists('temp'):
            os.makedirs('temp')

        # Camera settings
        self.camera_width = 320  
        self.camera_height = 240
        self.camera_fov = 60
        self.camera_aspect = self.camera_width / self.camera_height
        self.camera_near = 0.1
        self.camera_far = 100.0

        self.current_movement = 'stop'
        self.current_speed = 0
        self.left_wheel_velocity = 0
        self.right_wheel_velocity = 0
            
        # Setup GUI and start simulation
        self.setup_debug_gui()
        self.simulation_running = True
        self.start_simulation()

    def create_flat_ground(self):
        """Create a custom flat ground plane without grid pattern"""
        # Create a temporary URDF file for the flat ground
        ground_urdf = """<?xml version="1.0"?>
        <robot name="flat_ground">
            <link name="ground">
                <visual>
                    <geometry>
                        <box size="100 100 0.02"/>
                    </geometry>
                    <material name="green">
                        <color rgba="0.2 0.8 0.2 1"/>
                    </material>
                </visual>
                <collision>
                    <geometry>
                        <box size="100 100 0.02"/>
                    </geometry>
                </collision>
                <inertial>
                    <mass value="0"/>
                    <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
                </inertial>
            </link>
        </robot>
        """
            
        # Save the URDF to a temporary file
        if not os.path.exists('temp'):
            os.makedirs('temp')
        with open('temp/flat_ground.urdf', 'w') as f:
            f.write(ground_urdf)

        # Load the custom ground plane
        self.plane = p.loadURDF('temp/flat_ground.urdf', [0, 0, -0.01], useFixedBase=1)

    def create_obstacles(self):
        """Create a cube 10 units away from center"""
        cube_size = 1.0
        cube_mass = 1.0
        cube_position = [3, 0, cube_size/2]  # 10 units away on x-axis, raised by half its height
        
        # Create collision shape
        cube_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_size/2]*3)
        
        # Create visual shape with a distinct color
        cube_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[cube_size/2]*3, rgbaColor=[1, 0, 0, 1])  # Red cube
        
        # Create cube body
        self.cube = p.createMultiBody(
            baseMass=cube_mass,
            baseCollisionShapeIndex=cube_collision,
            baseVisualShapeIndex=cube_visual,
            basePosition=cube_position
        )

    def parse_urdf_for_wheels(self, urdf_path):
        """Parse URDF file to find wheel joints"""
        try:
            tree = ET.parse(urdf_path)
            root = tree.getroot()
            
            # Find all joint elements
            joints = root.findall('.//joint')
            
            # Look for wheel joints
            wheel_joints = []
            for joint in joints:
                joint_name = joint.get('name', '')
                joint_type = joint.get('type', '')
                
                # Look for common wheel joint patterns in the name
                if (joint_type in ['continuous', 'revolute'] and 
                    ('wheel' in joint_name.lower())): 
                    wheel_joints.append(joint_name)

            print("Setting joint", wheel_joints)
            
            return wheel_joints
        except ET.ParseError as e:
            print(f"Error parsing URDF: {e}")
            return []
        
    def find_joint_indices(self):
        """Get joint indices and information from loaded robot"""
        num_joints = p.getNumJoints(self.robot)
        wheel_joints = {}
        
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot, i)
            joint_name = joint_info[1].decode('utf-8')
            
            # Check if this joint is in our wheel joints list
            if ('wheel' in joint_name.lower()):
                
                # Store joint information
                wheel_joints[joint_name] = {
                    'index': i,
                    'type': joint_info[2],
                    'lowerLimit': joint_info[8],
                    'upperLimit': joint_info[9],
                    'maxForce': joint_info[10],
                    'maxVelocity': joint_info[11]
                }
                
                # Try to determine if it's left or right based on name
                if 'left' in joint_name.lower():
                    wheel_joints[joint_name]['side'] = 'left'
                elif 'right' in joint_name.lower():
                    wheel_joints[joint_name]['side'] = 'right'
                else:
                    # If side is not clear from name, try to determine from position
                    joint_pos = p.getLinkState(self.robot, i)[0]
                    wheel_joints[joint_name]['side'] = 'left' if joint_pos[1] > 0 else 'right'
        
        return wheel_joints

    def setup_debug_gui(self):
        # Store debug parameter IDs
        self.debug_params = {}
        
        # Create upload button first
        self.debug_params['upload'] = p.addUserDebugParameter("Upload URDF File", 1, 0, 0)
        
        # Create control buttons
        self.debug_params['forward'] = p.addUserDebugParameter("Forward", 1, 0, 0)
        self.debug_params['backward'] = p.addUserDebugParameter("Backward", 1, 0, 0)
        self.debug_params['left'] = p.addUserDebugParameter("Turn Left", 1, 0, 0)
        self.debug_params['right'] = p.addUserDebugParameter("Turn Right", 1, 0, 0)
        self.debug_params['stop'] = p.addUserDebugParameter("Stop", 1, 0, 0)
        self.debug_params['speed'] = p.addUserDebugParameter("Speed", -100, 100, 10.0)
        
        # Store previous button states
        self.prev_button_states = {key: 0 for key in self.debug_params}

        # Add instruction text
        self.text_id = p.addUserDebugText(
            text="Click 'Upload URDF File' button to load a robot",
            textPosition=[0, 0, 1],
            textColorRGB=[1, 1, 1],
            textSize=1.5
        )

    def handle_file_upload(self):
        root = tk.Tk()
        root.withdraw()
        
        file_path = filedialog.askopenfilename(
            title="Select URDF File",
            filetypes=[("URDF files", "*.urdf"), ("All files", "*.*")]
        )
        
        root.destroy()
        
        if file_path:
            try:
                # Parse URDF first to check for wheel joints
                wheel_joints = self.parse_urdf_for_wheels(file_path)
                if not wheel_joints:
                    raise Exception("No wheel joints found in URDF")

                # Copy the URDF file to temp directory
                filename = os.path.basename(file_path)
                urdf_path = os.path.join('temp', filename)
                shutil.copy2(file_path, urdf_path)
                
                # Remove existing robot if any
                if self.robot is not None:
                    p.removeBody(self.robot)
                
                # Add temp directory to search path and load new robot
                p.setAdditionalSearchPath('temp')
                self.robot = p.loadURDF(filename, [0, 0, 0.2], useFixedBase=False)
                
                # Get wheel joint information
                self.wheel_joints = self.find_joint_indices()
                
                if not self.wheel_joints:
                    raise Exception("Could not identify wheel joints in loaded robot")
                
                # Display joint information
                info_text = f"Loaded: {filename}\nWheel joints found:\n"
                # for name, info in self.wheel_joints.items():
                #     info_text += f"- {name} ({info['side']})\n"
                
                if hasattr(self, 'text_id'):
                    p.removeUserDebugItem(self.text_id)
                self.text_id = p.addUserDebugText(
                    text=info_text,
                    textPosition=[0, 0, 1],
                    textColorRGB=[0, 1, 0],
                    textSize=1.2
                )
                
            except Exception as e:
                print(f"Error loading URDF: {e}")
                if hasattr(self, 'text_id'):
                    p.removeUserDebugItem(self.text_id)
                self.text_id = p.addUserDebugText(
                    text=f"Error: {str(e)}",
                    textPosition=[0, 0, 1],
                    textColorRGB=[1, 0, 0],
                    textSize=1.2
                )

    def check_button_pressed(self, param_name):
        current_state = p.readUserDebugParameter(self.debug_params[param_name])
        if current_state != self.prev_button_states[param_name]:
            self.prev_button_states[param_name] = current_state
            return True
        return False
    
    def publish_twist_message(self, linear_x, angular_z):
        """Publish Twist message based on robot movement"""
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = float(angular_z)
        
        self.ros_node.publisher.publish(twist_msg)
        print(f'Published Twist - linear.x: {linear_x}, angular.z: {angular_z}')

    def set_wheel_velocities(self, left_vel, right_vel):
        """Set velocities for all wheels based on side"""
        if not self.wheel_joints:
            return
        
        # Store current velocities
        self.left_wheel_velocity = left_vel
        self.right_wheel_velocity = right_vel

        # Calculate linear and angular velocities for Twist message
        wheel_radius = 0.0625  # Adjust based on your robot's wheel size
        wheel_separation = 0.26  # Adjust based on your robot's wheel separation
        sim2real_ratio = 0.5
        # Calculate linear and angular velocities
        linear_x = sim2real_ratio * wheel_radius * (right_vel + left_vel) / 2.0
        angular_z = sim2real_ratio * wheel_radius * (left_vel - right_vel) / wheel_separation
        
        # Publish Twist message
        self.publish_twist_message(linear_x, angular_z)
        
        for joint_info in self.wheel_joints.values():
            # Get the exact velocity based on wheel side
            velocity = left_vel if joint_info['side'] == 'left' else right_vel
                
            p.setJointMotorControl2(
                self.robot,
                joint_info['index'],
                p.VELOCITY_CONTROL,
                targetVelocity=velocity
            )

    def update_robot_movement(self):
        if self.robot is None or not self.wheel_joints:
            return
            
        speed = p.readUserDebugParameter(self.debug_params['speed'])
        self.current_speed = speed

        # Track previous movement before updating
        previous_movement = self.current_movement

        if self.check_button_pressed('forward'):
            self.current_movement = 'forward'
            self.set_wheel_velocities(speed, speed)
        elif self.check_button_pressed('backward'):
            self.current_movement = 'backward'
            self.set_wheel_velocities(-speed, -speed)
        elif self.check_button_pressed('left'):
            self.current_movement = 'left'
            self.set_wheel_velocities(-0.25 * speed, 0.25 * speed)
        elif self.check_button_pressed('right'):
            self.current_movement = 'right'
            self.set_wheel_velocities(0.25 * speed, -0.25 * speed)
        elif self.check_button_pressed('stop'):
            self.current_movement = 'stop'
            # Ensure both wheels come to a complete stop
            self.set_wheel_velocities(0, 0)
        
        # If movement hasn't changed, maintain current velocities
        if previous_movement == self.current_movement and self.current_movement != 'stop':
            self.set_wheel_velocities(self.left_wheel_velocity, self.right_wheel_velocity)

        # Process any pending ROS callbacks
        rclpy.spin_once(self.ros_node, timeout_sec=0)

    def get_camera_view(self):
        """Update camera view from robot's perspective"""
        if self.robot is None:
            return

        # Get the robot's position and orientation
        robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot)
        
        # Calculate camera position (slightly above and forward of robot)
        camera_height = 0.3
        camera_forward = 0.2
        
        # Convert robot orientation to rotation matrix
        rot_matrix = p.getMatrixFromQuaternion(robot_orn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        
        # Calculate forward vector
        forward = np.array([rot_matrix[0][0], rot_matrix[1][0], rot_matrix[2][0]])
        
        # Calculate camera position
        camera_pos = np.array(robot_pos) + np.array([0, 0, camera_height])
        camera_pos = camera_pos + forward * camera_forward
        
        # Calculate target position (looking forward)
        target_pos = camera_pos + forward * 1.0
        
        # Calculate up vector
        up_vector = [0, 0, 1]
        
        # Compute view matrix
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_pos,
            cameraTargetPosition=target_pos,
            cameraUpVector=up_vector
        )
        
        # Compute projection matrix
        proj_matrix = p.computeProjectionMatrixFOV(
            fov=self.camera_fov,
            aspect=self.camera_aspect,
            nearVal=self.camera_near,
            farVal=self.camera_far
        )
        
        # Get camera image for the synthetic camera panel
        p.getCameraImage(
            width=self.camera_width,
            height=self.camera_height,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

    def simulation_loop(self):
        while self.simulation_running:
            if self.check_button_pressed('upload'):
                self.handle_file_upload()
            
            self.update_robot_movement()
            self.get_camera_view()
            p.stepSimulation()
            time.sleep(0.00001)

    def start_simulation(self):
        self.simulation_thread = threading.Thread(target=self.simulation_loop)
        self.simulation_thread.daemon = True
        self.simulation_thread.start()

    def cleanup(self):
        self.simulation_running = False
        if hasattr(self, 'simulation_thread'):
            self.simulation_thread.join()
        p.disconnect()
        if os.path.exists('temp'):
            shutil.rmtree('temp')
        self.ros_node.destroy_node()
        rclpy.shutdown()

def main():
    sim = None
    try:
        sim = RobotSimulation()
        while True:
            time.sleep(0.00001)
    except KeyboardInterrupt:
        print("\nShutting down simulation...")
    finally:
        if sim:
            sim.cleanup()
        print("Simulation ended successfully")

if __name__ == "__main__":
    main()