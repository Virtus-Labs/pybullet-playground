
import pybullet as p
import pybullet_data
import numpy as np
import math
from PIL import Image
import os

class PhysicsSimulator:
    def __init__(self, canvas_width=800, canvas_height=600):
        if canvas_width <= 1 or canvas_height <= 1:
            canvas_width = 800
            canvas_height = 600

        self.width = canvas_width
        self.height = canvas_height
        self.running = True
        self.loaded_models = [] 
        
        # Initialize physics engine
        print("Initializing Physics Simulator...")
        self.physics_init()
        
        # Camera parameters
        self.distance = 2
        self.yaw = 45
        self.pitch = 45
        self.target = [0.15, 0, 0]
        self.up_vector = [0, 0, 1]
        self.update_camera()

    def physics_init(self):
        """Initialize physics engine"""
        try:
            print("Connecting to PyBullet...")
            self.physics_client = p.connect(p.DIRECT)
            print(f"Connected to PyBullet with client ID: {self.physics_client}")
            
            print("Setting up PyBullet environment...")
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            print(f"PyBullet data path: {pybullet_data.getDataPath()}")
            
            p.setGravity(0, 0, -9.81)
            print("Gravity set to -9.81 m/s²")
            
            # Load ground plane
            print("Loading ground plane...")
            self.plane_id = p.loadURDF("plane.urdf")
            if self.plane_id >= 0:
                print(f"Ground plane loaded successfully with ID: {self.plane_id}")
                p.changeVisualShape(self.plane_id, -1, rgbaColor=[0, 1, 0, 1])
            else:
                print("Failed to load ground plane")
            # Enable debugging
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        except Exception as e:
            print(f"Error initializing physics: {e}")
            import traceback
            traceback.print_exc()
            self.running = False

    def load_urdf(self, urdf_path):
        """Load a URDF file into the simulation"""
        try:
            print(f"\nAttempting to load URDF from: {urdf_path}")
            
            # Verify file exists
            if not os.path.exists(urdf_path):
                print(f"Error: URDF file does not exist at path: {urdf_path}")
                return False
                
            # Clear existing models except ground plane
            print("Clearing existing models...")
            self.clear_models()
            
            # Load the cube above the ground
            start_pos = [0, 0, 0.2]
            start_orn = p.getQuaternionFromEuler([0, 0, 0])
            
            print(f"Loading URDF with parameters:")
            print(f"  Position: {start_pos}")
            print(f"  Orientation: {start_orn}")
            
            try:
                # Try to load the URDF
                model_id = p.loadURDF(
                    urdf_path,
                    start_pos,
                    start_orn,
                    useFixedBase=False,
                    physicsClientId=self.physics_client
                )
            except p.error as e:
                print(f"PyBullet error loading URDF: {e}")
                return False

            if model_id is not None and model_id >= 0:
                print(f"Successfully loaded URDF with ID: {model_id}")
                self.loaded_models.append(model_id)
                
                # Print model info
                info = p.getBodyInfo(model_id)
                print(f"Model info: {info}")
                
                # Print joint info
                num_joints = p.getNumJoints(model_id)
                print(f"Number of joints: {num_joints}")
                
                return True
            else:
                print("Failed to load URDF: Invalid model ID returned")
                return False
                
        except Exception as e:
            print(f"Error loading URDF file: {e}")
            import traceback
            traceback.print_exc()
            return False

        
    def add_default_dynamics(self, model_id):
        """Add default dynamic properties to the model"""
        try:
            num_joints = p.getNumJoints(model_id)
            for i in range(num_joints):
                # Add some mass and inertia to each link
                p.changeDynamics(model_id, i,
                    mass=1.0,
                    lateralFriction=0.5,
                    spinningFriction=0.1,
                    rollingFriction=0.1,
                    restitution=0.1,
                    linearDamping=0.1,
                    angularDamping=0.1)
                
                # Set joint motors for the wheels
                joint_info = p.getJointInfo(model_id, i)
                joint_name = joint_info[1].decode('utf-8')
                if 'wheel' in joint_name.lower():
                    p.setJointMotorControl2(
                        model_id, i,
                        p.VELOCITY_CONTROL,
                        targetVelocity=0,
                        force=10
                    )
                    
        except Exception as e:
            print(f"Error adding dynamics: {e}")
        
    def clear_models(self):
        """Remove all loaded models from simulation"""
        try:
            print(f"Clearing {len(self.loaded_models)} models...")
            for model_id in self.loaded_models:
                p.removeBody(model_id)
            self.loaded_models.clear()
            print("Models cleared successfully")
        except Exception as e:
            print(f"Error clearing models: {e}")

    def stepSimulation(self):
        """Single step of physics simulation"""
        if self.running:
            try:
                p.stepSimulation(physicsClientId=self.physics_client)
                
                # Print positions of all objects for debugging
                if self.loaded_models:
                    for model_id in self.loaded_models:
                        pos, orn = p.getBasePositionAndOrientation(model_id)
                        #print(f"Model {model_id} position: {pos}")
                        
            except Exception as e:
                print(f"Step simulation error: {e}")
                self.running = False

    def get_frame(self):
        """Get current frame from simulation"""
        try:
            if not self.running:
                return None
                
            # Get rendering from PyBullet
            width, height, rgba, depth, seg = p.getCameraImage(
                width=self.width,
                height=self.height,
                viewMatrix=self.view_matrix,
                projectionMatrix=self.proj_matrix,
                renderer=p.ER_BULLET_HARDWARE_OPENGL,
                physicsClientId=self.physics_client
            )
            
            # Convert to PIL Image
            rgba_array = np.reshape(rgba, (height, width, 4))
            rgb_array = rgba_array[:, :, :3]
            return Image.fromarray(rgb_array)
        except Exception as e:
            print(f"Get frame error: {e}")
            import traceback
            traceback.print_exc()
            self.running = False
            return None

    def stop(self):
        """Stop simulation and cleanup"""
        self.running = False
        try:
            p.disconnect(physicsClientId=self.physics_client)
        except Exception as e:
            print(f"Disconnect error: {e}")

    def zoom(self, factor):
        self.distance = max(0.1, min(20, self.distance * factor))
        self.update_camera()

    def rotate(self, dyaw, dpitch):
        self.yaw += dyaw
        self.pitch = max(-89, min(89, self.pitch + dpitch))
        self.update_camera()

    def update_camera(self):
        """Update camera position"""
        try:
            pos = [
                self.target[0] + self.distance * math.cos(math.radians(self.yaw)) * math.cos(math.radians(self.pitch)),
                self.target[1] + self.distance * math.sin(math.radians(self.yaw)) * math.cos(math.radians(self.pitch)),
                self.target[2] + self.distance * math.sin(math.radians(self.pitch))
            ]
            self.view_matrix = p.computeViewMatrix(pos, self.target, self.up_vector)
            self.proj_matrix = p.computeProjectionMatrixFOV(60, float(self.width)/self.height, 0.1, 100.0)
        except Exception as e:
            print(f"Camera update error: {e}")

    def reset_camera_view(self):
        """Reset camera to view the model"""
        if self.loaded_models:
            try:
                # Get the AABB of the model
                aabb_min, aabb_max = p.getAABB(self.loaded_models[0])
                
                # Calculate center point
                center = [
                    (aabb_min[0] + aabb_max[0]) / 2,
                    (aabb_min[1] + aabb_max[1]) / 2,
                    (aabb_min[2] + aabb_max[2]) / 2
                ]
                
                # Update target and distance based on model size
                self.target = center
                size = max(aabb_max[i] - aabb_min[i] for i in range(3))
                self.distance = size * 3.0  # Adjust multiplier as needed
                
                # Update camera
                self.update_camera()
                
            except Exception as e:
                print(f"Error resetting camera view: {e}")