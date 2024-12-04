class ROSPyBulletBridge:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.simulator = None
        self.robot_id = None
        
    def start(self):
        """Initialize PyBullet simulator"""
        if self.simulator is None:
            import pybullet as p
            import pybullet_data
            
            # Start PyBullet in GUI mode
            self.simulator = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            
            # Configure simulator
            p.setGravity(0, 0, -9.81)
            p.setRealTimeSimulation(1)
            
            # Load ground plane
            p.loadURDF("plane.urdf")
            
            # Set camera
            p.resetDebugVisualizerCamera(
                cameraDistance=2.0,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[0, 0, 0]
            )
            
            return True
        return False
        
    def load_robot_from_description(self, robot_description):
        """Load robot from URDF string into PyBullet"""
        if self.simulator is None:
            return False
            
        try:
            import pybullet as p
            import tempfile
            
            # Create temporary URDF file
            with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
                f.write(robot_description)
                urdf_path = f.name
            
            # Remove existing robot if any
            if self.robot_id is not None:
                p.removeBody(self.robot_id)
            
            # Load new robot
            self.robot_id = p.loadURDF(urdf_path, [0, 0, 0])
            
            # Clean up temp file
            import os
            os.unlink(urdf_path)
            
            return True
            
        except Exception as e:
            print(f"Error loading robot: {e}")
            return False
            
    def stop(self):
        """Stop PyBullet simulator"""
        if self.simulator is not None:
            import pybullet as p
            p.disconnect()
            self.simulator = None
            self.robot_id = None