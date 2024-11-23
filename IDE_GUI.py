import tkinter as tk
from tkinter import ttk
import tkinter.font as tkfont
import pybullet as p
import pybullet_data
import time
import os
import subprocess
import threading
import queue
import pty
import select
import termios
import tty
import fcntl
import struct
from PIL import Image, ImageTk
import numpy as np
import math
import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import xml.etree.ElementTree as ET
import tempfile

from tkinter import filedialog
from physics_simulation import PhysicsSimulator
from ros_pybullet_bridge import ROSPyBulletBridge

class RobotLoader(Node):
    def __init__(self):
        super().__init__('robot_loader')
        self.description_sub = self.create_subscription(
            String,
            'robot_description',
            self.description_callback,
            10)
        self.description_queue = queue.Queue()

    def description_callback(self, msg):
        print(msg.data)
        self.description_queue.put(msg.data)

class IDEApp:
    def __init__(self, root):
        # Initialize root window and basic properties
        self.root = root
        self.root.title("Rigit IDE")
        self.folder_name = "MyProject"
        
        # Initialize file tracking
        self.current_file = None
        self.file_contents = {}
        self.launch_file_paths = {}
        
        # Initialize simulation properties
        self.simulator = None
        self.sim_thread = None
        self.render_thread = None

        self.pybullet_connected = False
        self.robot_loaded = False
        self.ros_node = None
        self.robot_loader = None
        
        # Configure colors
        self.bg_dark = "#1e1e1e"
        self.bg_darker = "#252526"
        self.bg_sidebar = "#333333"
        self.bg_lighter = "#2d2d2d"
        self.text_color = "#d4d4d4"
        self.text_color_dim = "#858585"
        self.accent_color = "#3c8039"
        self.select_color = "#1f538d"
        self.button_color = "#0096FF"
        
        # Configure styles
        self.style = ttk.Style()
        self.style.theme_use('default')
        
        # Configure Treeview style
        self.style.configure("Treeview",
                        background=self.bg_darker,
                        foreground=self.text_color,
                        fieldbackground=self.bg_darker)
        self.style.map('Treeview',
                    background=[('selected', self.select_color)],
                    foreground=[('selected', 'white')])
        
        # Configure button style
        self.style.configure("Header.TButton",
                        background=self.bg_darker,
                        foreground=self.text_color,
                        borderwidth=0,
                        padding=5)
        
        # Configure main window
        self.root.configure(bg=self.bg_dark)
        self.root.grid_rowconfigure(1, weight=1)
        self.root.grid_columnconfigure(1, weight=1)
        
        # Create main UI elements in correct order
        self.create_toolbar()
        
        # Create main container
        self.main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.main_paned.grid(row=1, column=1, sticky="nsew", padx=1, pady=(1, 1))
        
        # Create editor and preview paned window
        self.editor_preview_paned = ttk.PanedWindow(self.main_paned, orient=tk.HORIZONTAL)
        
        # Create UI components
        self.create_file_explorer()     # Creates self.sidebar_frame
        self.create_code_editor()       # Creates self.editor_frame
        self.create_simulator_preview() # Creates self.preview_frame
        self.create_terminal()          # Creates terminal
        
        # Pack components
        self.main_paned.add(self.sidebar_frame, weight=1)
        self.main_paned.add(self.editor_preview_paned, weight=3)
        self.editor_preview_paned.add(self.editor_frame, weight=1)
        
        # Bind configure events for layout management
        self.root.bind("<Configure>", self.on_window_configure)
        
        # Initial layout update
        self.root.update_idletasks()
        self.initial_layout = False  # Flag to track initial layout
        # self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def __del__(self):
        self.stop_simulation()

    def on_window_configure(self, event=None):
        """Handle window configuration (resize) events"""
        if event.widget == self.root:
            if not hasattr(self, 'initial_layout') or not self.initial_layout:
                self.set_paned_proportions()
                self.initial_layout = True
        
    def set_paned_proportions(self):
        """Set the initial proportions of the paned windows"""
        try:
            self.root.update_idletasks()
            
            # Get the actual window width
            total_width = self.root.winfo_width()
            if total_width <= 1:  # Window not properly initialized yet
                self.root.after(100, self.set_paned_proportions)
                return
                
            # Calculate desired widths
            explorer_width = int(total_width * 0.25)
            
            # Configure main paned window
            try:
                self.main_paned.sashpos(0, explorer_width)
            except Exception as e:
                print(f"Main paned window not ready yet: {e}")
                return
                
            # Configure editor/preview split if preview is visible
            if hasattr(self, 'preview_frame') and self.preview_frame.winfo_ismapped():
                remaining_width = total_width - explorer_width
                editor_width = int(remaining_width * 0.6)
                try:
                    self.editor_preview_paned.sashpos(0, editor_width)
                except Exception as e:
                    print(f"Editor preview paned window not ready yet: {e}")
                    
        except Exception as e:
            print(f"Error in set_paned_proportions: {e}")
            
    def create_toolbar(self):
        # Top toolbar
        toolbar = tk.Frame(self.root, bg=self.bg_darker, height=30)
        toolbar.grid(row=0, column=1, sticky="ew")
        toolbar.grid_columnconfigure(1, weight=1)  # For spacing between left and right items
        
        # Left side of toolbar
        left_tools = tk.Frame(toolbar, bg=self.bg_darker)
        left_tools.grid(row=0, column=0, sticky="w")

        # Create a container for icon and label
        icon_container = tk.Frame(left_tools, bg=self.bg_darker)
        icon_container.pack(side=tk.LEFT, padx=5)

        # Project icon using Canvas
        icon_size = 40
        self.icon_canvas = tk.Canvas(icon_container, 
                            width=icon_size, 
                            height=icon_size,
                            bg=self.bg_darker,
                            highlightthickness=0)
        self.icon_canvas.pack(side=tk.TOP)

        # Draw rounded rectangle with proper corners
        def create_rounded_rect(canvas, x1, y1, x2, y2, radius):
            # Create points for a rounded rectangle
            points = [
                x1 + radius, y1,                # Top left after curve
                x2 - radius, y1,                # Top right before curve
                x2, y1,                         # Top right start curve
                x2, y1 + radius,                # Top right after curve
                x2, y2 - radius,                # Bottom right before curve
                x2, y2,                         # Bottom right start curve
                x2 - radius, y2,                # Bottom right after curve
                x1 + radius, y2,                # Bottom left before curve
                x1, y2,                         # Bottom left start curve
                x1, y2 - radius,                # Bottom left after curve
                x1, y1 + radius,                # Top left before curve
                x1, y1                          # Top left start curve
            ]
            return canvas.create_polygon(points, smooth=True, fill=self.button_color)

        # Create the rounded rectangle button with all corners rounded
        create_rounded_rect(self.icon_canvas, 2, 2, icon_size-2, icon_size-2, 4)
    
        # Add text
        self.icon_canvas.create_text(icon_size/2, icon_size/2, 
                            text=self.get_folder_icon(self.folder_name), 
                            fill=self.text_color,
                            font=('Arial', 10, 'bold'))

        
        # Project combobox
        project_combo = ttk.Combobox(left_tools, values=["New Project...", "Open Project...", "Close Project"],
                                    style="Toolbar.TCombobox", width=15)
        project_combo.set(self.folder_name)
        project_combo.pack(side=tk.LEFT, padx=(2, 10))
        project_combo.bind('<<ComboboxSelected>>', self.handle_project_actions)

        # Version Control combobox
        # vc_combo = ttk.Combobox(left_tools, values=["Git Operations", "Commit", "Push", "Pull"],
        #                     style="Toolbar.TCombobox", width=20)
        # vc_combo.set("Version Control")
        # vc_combo.pack(side=tk.LEFT, padx=2)

         # Right side of toolbar
        right_tools = tk.Frame(toolbar, bg=self.bg_darker)
        right_tools.grid(row=0, column=2, sticky="e")
        
        # Raspberry Pi combobox
        # pi_combo = ttk.Combobox(right_tools, values=["Connect", "Disconnect", "Settings"],
        #                     style="Toolbar.TCombobox", width=15)
        # pi_combo.set("🤖 Raspberry Pi")
        # pi_combo.pack(side=tk.LEFT, padx=2)
        
        # Build button
        # build_btn = ttk.Button(right_tools, text="Build", style="Toolbar.TButton")
        # build_btn.pack(side=tk.LEFT, padx=2)
        
        # Run combobox
        self.run_files = self.get_launch_files()
        self.run_combo = ttk.Combobox(right_tools, 
                                     values=self.run_files,
                                     style="Toolbar.TCombobox", 
                                     width=50)
        self.run_combo.set("main.py")
        self.run_combo.pack(side=tk.LEFT, padx=2)
        self.run_combo.bind('<<ComboboxSelected>>', self.on_launch_file_selected)
        
        # Play button
        self.run_btn_frame = tk.Frame(right_tools, bg=self.bg_darker)
        self.run_btn_frame.pack(side=tk.LEFT, padx=2)
        
        self.run_btn = ttk.Button(self.run_btn_frame, 
                                text="▶", 
                                style="Toolbar.TButton",
                                command=self.toggle_run_state)
        self.run_btn.pack(expand=True, fill=tk.BOTH)
        
        # Initialize running state
        self.is_running = False
        self.current_process = None
        self.process_pid = None

        # self.control_btn = ttk.Button(right_tools, text="Teloperate", 
        #                             style="Toolbar.TButton",
        #                             command=self.toggle_control_panel)
        # self.control_btn.pack(side=tk.LEFT, padx=2)
        
        # Icons
        # search_btn = ttk.Button(right_tools, text="Search", style="Toolbar.TButton")
        # settings_btn = ttk.Button(right_tools, text="Settings", style="Toolbar.TButton")
        # user_btn = ttk.Button(right_tools, text="Account", style="Toolbar.TButton")
        
        # search_btn.pack(side=tk.LEFT, padx=2)
        # settings_btn.pack(side=tk.LEFT, padx=2)
        # user_btn.pack(side=tk.LEFT, padx=2)

    def handle_project_actions(self, event):
        selected = event.widget.get()
        if selected == "Open Project...":
            folder_path = filedialog.askdirectory(title="Select Project Folder")
            if folder_path:
                self.load_project(folder_path)

    def load_project(self, folder_path):
        """Load a project directory into the file explorer"""
        # Store the project root path
        self.project_root = folder_path
        
        # Clear existing tree
        for item in self.tree.get_children():
            self.tree.delete(item)
            
        # Update folder name and icon
        self.folder_name = os.path.basename(folder_path)
        
        # Update project label
        for widget in self.sidebar_frame.winfo_children():
            if isinstance(widget, tk.Label) and widget.cget("font") == ("Arial", 11, "bold"):
                widget.config(text=self.folder_name)
        
        # Update toolbar project name
        for widget in self.root.winfo_children():
            if isinstance(widget, ttk.Combobox):
                widget.set(self.folder_name)
                
        # Add root folder
        root = self.tree.insert("", "end", text=self.folder_name, open=True)
        
        # Recursively add contents
        self.populate_directory(root, folder_path)
        
        # Update the icon
        self._update_project_icon()

        # Update run combobox with new launch files
        self.run_files = self.get_launch_files()
        self.run_combo['values'] = self.run_files
        
        # Reset to main.py if it exists, otherwise first available file
        if "main.py" in self.run_files:
            self.run_combo.set("main.py")
        elif self.run_files and self.run_files[0] != "No launch files found":
            self.run_combo.set(self.run_files[0])
        else:
            self.run_combo.set("Select Launch File")

    def _update_project_icon(self):
        # Clear previous icon
        self.icon_canvas.delete("all")
        
        # Redraw icon with new text
        icon_size = 40
        radius = 4

        # Recreate rounded rectangle
        points = [
            2 + radius, 2,                    # Top left after curve
            icon_size-2 - radius, 2,          # Top right before curve
            icon_size-2, 2,                   # Top right start curve
            icon_size-2, 2 + radius,          # Top right after curve
            icon_size-2, icon_size-2 - radius, # Bottom right before curve
            icon_size-2, icon_size-2,         # Bottom right start curve
            icon_size-2 - radius, icon_size-2, # Bottom right after curve
            2 + radius, icon_size-2,          # Bottom left before curve
            2, icon_size-2,                   # Bottom left start curve
            2, icon_size-2 - radius,          # Bottom left after curve
            2, 2 + radius,                    # Top left before curve
            2, 2                              # Top left start curve
        ]
        self.icon_canvas.create_polygon(points, smooth=True, fill=self.button_color)
        
        # Update icon text
        self.icon_canvas.create_text(icon_size/2, icon_size/2,
                                text=self.get_folder_icon(self.folder_name),
                                fill=self.text_color,
                                font=('Arial', 10, 'bold'))
        
        # Update project name label
        project_display_name = self.folder_name.replace('-', ' ').replace('_', ' ').upper()
        self.project_label.config(text=project_display_name)

    def populate_directory(self, parent, path):
        """Recursively populate the file explorer tree"""
        try:
            # Get all items in the directory
            items = os.listdir(path)
            
            # Separate folders and files
            folders = []
            files = []
            for item in items:
                if item.startswith('.'):  # Skip hidden files
                    continue
                item_path = os.path.join(path, item)
                if os.path.isdir(item_path):
                    folders.append((item, item_path))
                else:
                    files.append((item, item_path))
            
            # Sort folders and files alphabetically
            folders.sort()
            files.sort()
            
            # Add folders first
            for folder_name, folder_path in folders:
                folder_node = self.tree.insert(parent, "end", text=folder_name, values=(folder_path,))
                self.populate_directory(folder_node, folder_path)
            
            # Then add files
            for file_name, file_path in files:
                self.tree.insert(parent, "end", text=file_name, values=(file_path,))
                
        except PermissionError as e:
            print(f"Permission error accessing {path}: {str(e)}")
        except Exception as e:
            print(f"Error populating directory {path}: {str(e)}")

    def create_file_explorer(self):
        """Create the file explorer sidebar"""
        # Sidebar frame
        self.sidebar_frame = tk.Frame(self.main_paned, bg=self.bg_darker)
        # self.main_paned.add(self.sidebar_frame)
        
        # Project header frame
        header_frame = tk.Frame(self.sidebar_frame, bg=self.bg_darker)
        header_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Project label
        self.project_label = tk.Label(header_frame, text=self.folder_name,
                            bg=self.bg_darker, fg=self.text_color,
                            font=('Arial', 11, 'bold'))
        self.project_label.pack(side=tk.LEFT, padx=5)
        
        # Create tree with full row selection and stored values
        self.tree = ttk.Treeview(self.sidebar_frame, show='tree', selectmode='browse')
        self.tree.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Bind single click event
        self.tree.bind('<<TreeviewSelect>>', self.on_file_click)

    def get_folder_icon(self, folder_name):
        self.folder_name = folder_name
        words = folder_name.replace('-', ' ').replace('_', ' ').split()
        if len(words) >= 2:
            return (words[0][0] + words[1][0]).upper()
        return folder_name[:2].upper()

    def create_code_editor(self):
        """Create the code editor pane with multi-tab support"""
        # Editor frame
        self.editor_frame = tk.Frame(self.editor_preview_paned, bg=self.bg_dark)
        # self.editor_preview_paned.add(self.editor_frame)
        
        # Tab container
        self.tab_container = tk.Frame(self.editor_frame, bg=self.bg_darker)
        self.tab_container.pack(fill=tk.X)
        
        # Dictionary to store tabs and their content
        self.tabs = {}  # {filepath: {'tab': tab_frame, 'editor': editor, 'label': label}}
        self.active_tab = None
        
        # Editor container for the actual editor widgets
        self.editor_container = tk.Frame(self.editor_frame, bg=self.bg_dark)
        self.editor_container.pack(fill=tk.BOTH, expand=True)

    def start_simulation(self):
        """Start the physics simulation in a non-blocking way"""
        try:
            if self.simulator is None:
                width = self.sim_canvas.winfo_width()
                height = self.sim_canvas.winfo_height()
                
                if width > 1 and height > 1:
                    self.simulator = PhysicsSimulator(width, height)
                    if not self.simulator.running:
                        return
                    self.simulator_running = True  # Flag to control threads
                    
                    # Create and start simulation thread
                    self.sim_thread = threading.Thread(
                        target=self.simulation_loop,
                        daemon=True  # Make thread daemon so it exits with main program
                    )
                    
                    # Create and start render thread
                    self.render_thread = threading.Thread(
                        target=self.render_loop,
                        daemon=True  # Make thread daemon so it exits with main program
                    )
                    
                    self.sim_thread.start()
                    self.render_thread.start()
                    
        except Exception as e:
            print(f"Error starting simulation: {e}")
            self.stop_simulation()

    def simulation_loop(self):
        """Main simulation loop running in separate thread"""
        try:
            while self.simulator_running and self.simulator and self.simulator.running:
                self.simulator.stepSimulation()
                time.sleep(1/240)  # Simulation rate
        except Exception as e:
            print(f"Simulation error: {e}")

    def render_loop(self):
        """Render loop running in separate thread"""
        try:
            while self.simulator_running and self.simulator:
                if not self.sim_canvas.winfo_exists():
                    break
                    
                # Schedule frame update on main thread
                self.root.after_idle(self.update_frame)
                time.sleep(1/60)  # 60 FPS target
                
        except Exception as e:
            print(f"Render thread error: {e}")
            self.simulator_running = False

    def update_frame(self):
        """Update the canvas with the next frame (called on main thread)"""
        try:
            if not self.simulator or not self.simulator_running or not self.simulator.running:
                return
                
            frame = self.simulator.get_frame()
            if frame and self.sim_canvas.winfo_exists():
                canvas_width = self.sim_canvas.winfo_width()
                canvas_height = self.sim_canvas.winfo_height()
                if canvas_width > 1 and canvas_height > 1:
                    frame = frame.resize((canvas_width, canvas_height))
                    photo = ImageTk.PhotoImage(frame)
                    self.sim_canvas.delete("all")  # Clear previous frame
                    self.sim_canvas.create_image(0, 0, image=photo, anchor=tk.NW)
                    self.sim_canvas.image = photo  # Keep reference
        except Exception as e:
            print(f"Frame update error: {e}")
            self.simulator_running = False

    def stop_simulation(self):
        """Stop the simulation and cleanup threads"""
        try:
            # Signal threads to stop
            self.simulator_running = False
            
            # Stop physics engine
            if self.simulator:
                self.simulator.stop()
                self.simulator = None
            
            # Wait for threads to finish with timeout
            if hasattr(self, 'sim_thread') and self.sim_thread:
                self.sim_thread.join(timeout=1.0)
                self.sim_thread = None
                
            if hasattr(self, 'render_thread') and self.render_thread:
                self.render_thread.join(timeout=1.0)
                self.render_thread = None
                
        except Exception as e:
            print(f"Error stopping simulation: {e}")

    def create_simulator_preview(self):
        """Create the preview pane that will be shown/hidden for XML files"""
        preview_frame = tk.Frame(self.editor_preview_paned, bg=self.bg_darker)
        self.preview_frame = preview_frame  # Store reference to hide/show later
        preview_frame.grid_rowconfigure(1, weight=1)
        preview_frame.grid_columnconfigure(0, weight=1)

        # Header setup
        header_frame = tk.Frame(preview_frame, bg=self.bg_darker)
        header_frame.grid(row=0, column=0, sticky="ew")

        preview_label = tk.Label(header_frame, text="Preview",
                            bg=self.bg_darker, fg=self.text_color,
                            padx=10, pady=5)
        preview_label.pack(side=tk.LEFT)

        controls_frame = tk.Frame(header_frame, bg=self.bg_darker)
        controls_frame.pack(side=tk.RIGHT, padx=5)

        # self.start_btn = ttk.Button(controls_frame, text="Start Simulation", 
        #                     command=lambda: self.start_simulation())
        # self.start_btn.pack(side=tk.LEFT, padx=2)

        # stop_btn = ttk.Button(controls_frame, text="Stop Simulation", 
        #                     command=self.stop_simulation)
        # stop_btn.pack(side=tk.LEFT, padx=2)

        self.sim_canvas = tk.Canvas(preview_frame, bg=self.bg_darker)
        self.sim_canvas.grid(row=1, column=0, sticky="nsew", padx=0, pady=0)

        # Mouse controls
        self.last_x = 0
        self.last_y = 0
        self.sim_canvas.bind("<Button-1>", self.mouse_down)
        self.sim_canvas.bind("<B1-Motion>", self.mouse_move)
        self.sim_canvas.bind('<Button-4>', self.mouse_wheel)
        self.sim_canvas.bind('<Button-5>', self.mouse_wheel)
        self.sim_canvas.bind('<MouseWheel>', self.mouse_wheel)
        self.sim_canvas.bind('<Control-Button-4>', self.touchpad_zoom)
        self.sim_canvas.bind('<Control-Button-5>', self.touchpad_zoom)

        # Bind resize event
        self.sim_canvas.bind('<Configure>', self.on_canvas_resize)
        preview_frame.bind('<Configure>', self.on_preview_resize)

        # Initially hide the preview
        self.hide_preview()

    def show_preview_condition(self, filepath):
        """Determine if preview should be shown for this file type"""
        if not filepath:
            return False
            
        # Get filename and extension
        filename = os.path.basename(filepath)
        file_extension = os.path.splitext(filename)[1].lower()
        
        # Don't show preview for package.xml
        if filename.lower() == 'package.xml':
            return False
        
        # Show preview for .urdf and .xml files
        return file_extension in ['.xml','.urdf']

    def on_canvas_resize(self, event):
        """Handle canvas resize events"""
        if self.simulator and event.width > 1 and event.height > 1:
            self.simulator.width = event.width
            self.simulator.height = event.height
            self.simulator.update_camera()
            # Force aspect ratio update
            self.simulator.proj_matrix = p.computeProjectionMatrixFOV(
                60, float(event.width)/event.height, 0.1, 100.0
            )

    def on_preview_resize(self, event):
        """Handle preview frame resize events"""
        if event.width > 1 and event.height > 1:
            # Resize canvas to match preview frame
            header_height = self.preview_frame.grid_slaves(row=0, column=0)[0].winfo_height()
            canvas_height = event.height - header_height
            if canvas_height > 1:
                self.sim_canvas.configure(width=event.width, height=canvas_height)
                # Update simulator if it exists
                if self.simulator:
                    self.simulator.width = event.width
                    self.simulator.height = canvas_height
                    self.simulator.update_camera()
                    # Update projection matrix for new aspect ratio
                    self.simulator.proj_matrix = p.computeProjectionMatrixFOV(
                        60, float(event.width)/canvas_height, 0.1, 100.0
                    )

    def show_preview(self):
        """Show the preview pane"""
        if not hasattr(self, 'preview_frame'):
            return
            
        if not self.preview_frame.winfo_ismapped():
            self.editor_preview_paned.add(self.preview_frame, weight=1)
            self.root.update_idletasks()
            
            # Set preview width after adding it
            try:
                available_width = self.editor_preview_paned.winfo_width()
                editor_width = int(available_width * 0.6)
                if editor_width > 0:
                    self.editor_preview_paned.sashpos(0, editor_width)

                # Force a resize event to update the simulation
                self.preview_frame.event_generate('<Configure>', 
                    when='now',
                    width=self.preview_frame.winfo_width(),
                    height=self.preview_frame.winfo_height())

                self.start_simulation()

            except Exception as e:
                print(f"Error setting preview width: {e}")

    def hide_preview(self):
        """Hide the preview pane"""
        if hasattr(self, 'preview_frame') and self.preview_frame.winfo_ismapped():
            try:
                self.stop_simulation()  # Stop any running simulation
                self.editor_preview_paned.forget(self.preview_frame)
            except Exception as e:
                print(f"Error hiding preview: {e}")

    def mouse_down(self, event):
            self.last_x = event.x
            self.last_y = event.y

    def mouse_move(self, event):
        if self.simulator:
            dx = event.x - self.last_x
            dy = event.y - self.last_y
            self.simulator.rotate(dx * 0.5, -dy * 0.5)
            self.last_x = event.x
            self.last_y = event.y

    def touchpad_zoom(self, event):
        if self.simulator:
            factor = 1.1 if event.num == 4 else 0.9
            self.simulator.zoom(factor)
            
    def mouse_wheel(self, event):
        if self.simulator:
            if event.delta:  # Windows/macOS
                factor = 0.9 if event.delta < 0 else 1.1
            else:  # Linux
                factor = 0.9 if event.num == 5 else 1.1 if event.num == 4 else 1.0
            self.simulator.zoom(factor)

    def on_file_click(self, event):
        """Handle single-click selection events on the file tree"""
        selected_items = self.tree.selection()
        if not selected_items:
            return
            
        item_id = selected_items[0]
        item = self.tree.item(item_id)
        item_path = item['values'][0] if item['values'] else None
        
        # If it's a file and we have a path, open it
        if item_path and os.path.isfile(item_path):
            # Check if we should show preview
            if self.show_preview_condition(item_path):
                self.show_preview()
            else:
                self.hide_preview()
            
            # Open the file
            self.open_file(item_path)
    
    def get_item_path(self, item_id):
        """Get the full path of an item in the tree by traversing up to root"""
        current_dir = os.getcwd()  # Get current working directory
        path_parts = []
        
        # Traverse up the tree to build the path
        while item_id:
            item = self.tree.item(item_id)
            path_parts.insert(0, item['text'])
            item_id = self.tree.parent(item_id)
            
        # Build the full path
        return os.path.join(current_dir, *path_parts)
    
    def create_editor_widget(self, container):
        """Create a new editor widget with dynamic line numbers and scrollbars"""
        editor_frame = tk.Frame(container, bg=self.bg_dark)
        editor_frame.grid_rowconfigure(0, weight=1)
        editor_frame.grid_columnconfigure(1, weight=1)
        
        # Line numbers
        line_numbers = tk.Text(editor_frame, width=6, padx=5, pady=5,
                            bg=self.bg_dark, fg=self.text_color_dim,
                            bd=0, highlightthickness=0)
        line_numbers.grid(row=0, column=0, sticky="nsw")
        line_numbers.config(state='disabled')
        
        # Editor
        editor = tk.Text(editor_frame, wrap=tk.NONE, padx=5, pady=5,
                        bg=self.bg_dark, fg=self.text_color,
                        insertbackground=self.text_color,
                        bd=0, highlightthickness=0,
                        font=('Courier New', 10))
        editor.grid(row=0, column=1, sticky="nsew")
        
        # Vertical scrollbar
        v_scrollbar = ttk.Scrollbar(editor_frame, orient="vertical")
        v_scrollbar.grid(row=0, column=2, sticky="ns")
        
        # Horizontal scrollbar
        h_scrollbar = ttk.Scrollbar(editor_frame, orient="horizontal",
                                command=editor.xview)
        h_scrollbar.grid(row=1, column=1, sticky="ew")
        
        # Configure scrolling synchronization
        def on_editor_scroll(*args):
            line_numbers.yview_moveto(args[0])
            v_scrollbar.set(*args)
        
        def on_scrollbar_scroll(*args):
            line_numbers.yview(*args)
            editor.yview(*args)
        
        editor.config(yscrollcommand=on_editor_scroll,
                    xscrollcommand=h_scrollbar.set)
        v_scrollbar.config(command=on_scrollbar_scroll)
        
        # Sync line numbers scroll with mousewheel
        def on_mousewheel(event):
            if event.state & 0x1:  # Shift is pressed - horizontal scroll
                editor.xview_scroll(int(-1 * (event.delta / 120)), "units")
            else:  # Vertical scroll
                editor.yview_scroll(int(-1 * (event.delta / 120)), "units")
                line_numbers.yview_scroll(int(-1 * (event.delta / 120)), "units")
        
        editor.bind("<MouseWheel>", on_mousewheel)  # Windows
        editor.bind("<Button-4>", on_mousewheel)    # Linux up
        editor.bind("<Button-5>", on_mousewheel)    # Linux down
        
        # Bind line numbers update
        editor.bind('<<Modified>>', lambda e, ed=editor, ln=line_numbers: self.update_line_numbers(ed, ln))
        
        # Bind Ctrl+S to save
        editor.bind('<Control-s>', lambda e, ed=editor: self.save_file(ed))
        
        return editor_frame, editor, line_numbers

    def create_tab(self, filepath):
        """Create a new tab for the given file"""
        # Create tab frame
        tab_frame = tk.Frame(self.tab_container, bg=self.bg_darker)
        
        # Create tab label
        filename = os.path.basename(filepath)
        label = tk.Label(tab_frame, text=filename, bg=self.bg_darker,
                        fg=self.text_color, padx=10, pady=5)
        label.pack(side=tk.LEFT)
        
        # Create close button
        close_btn = tk.Label(tab_frame, text="×", bg=self.bg_darker,
                            fg=self.text_color, padx=5, pady=5,
                            cursor="hand2")
        close_btn.pack(side=tk.LEFT)
        
        # Create editor for this tab
        editor_frame, editor, line_numbers = self.create_editor_widget(self.editor_container)
        
        # Initially hide the editor frame
        editor_frame.pack_forget()
        
        # Store tab information
        self.tabs[filepath] = {
            'tab': tab_frame,
            'editor_frame': editor_frame,
            'editor': editor,
            'line_numbers': line_numbers,
            'label': label,
            'close_btn': close_btn
        }
        
        # Bind events
        tab_frame.bind('<Button-1>', lambda e, fp=filepath: self.switch_tab(fp))
        close_btn.bind('<Button-1>', lambda e, fp=filepath: self.close_tab(fp))
        label.bind('<Button-1>', lambda e, fp=filepath: self.switch_tab(fp))
        
        # Pack the tab
        tab_frame.pack(side=tk.LEFT, padx=(1, 0))
    
        return self.tabs[filepath]

    def switch_tab(self, filepath):
        """Switch to the specified tab"""
        if filepath not in self.tabs:
            return
            
        if self.active_tab:
            # Hide current active tab content
            self.tabs[self.active_tab]['editor_frame'].pack_forget()
            # Reset tab appearance
            self.tabs[self.active_tab]['tab'].configure(bg=self.bg_darker)
            self.tabs[self.active_tab]['label'].configure(bg=self.bg_darker)
            self.tabs[self.active_tab]['close_btn'].configure(bg=self.bg_darker)
        
        # Show new tab content
        self.tabs[filepath]['editor_frame'].pack(in_=self.editor_container, fill=tk.BOTH, expand=True)
        # Update tab appearance
        self.tabs[filepath]['tab'].configure(bg=self.bg_lighter)
        self.tabs[filepath]['label'].configure(bg=self.bg_lighter)
        self.tabs[filepath]['close_btn'].configure(bg=self.bg_lighter)
        
        self.active_tab = filepath

        # Show/hide preview based on file type
        if self.show_preview_condition(filepath):
            self.show_preview()
        else:
            self.hide_preview()

    def close_tab(self, filepath):
        """Close the specified tab"""
        if filepath in self.tabs:
            # If closing active tab, switch to another tab first
            if self.active_tab == filepath:
                # Find another tab to switch to
                other_tabs = [fp for fp in self.tabs.keys() if fp != filepath]
                if other_tabs:
                    self.switch_tab(other_tabs[0])
                else:
                    self.active_tab = None
                    self.hide_preview()
            
            # Destroy tab widgets
            self.tabs[filepath]['tab'].destroy()
            self.tabs[filepath]['editor_frame'].destroy()
            
            # Remove from tabs dictionary
            del self.tabs[filepath]

    def update_line_numbers(self, editor, line_numbers):
        """Update line numbers based on editor content"""
        if not editor.edit_modified():
            return
        
        # Get contents and count lines
        contents = editor.get("1.0", "end-1c")
        line_count = contents.count('\n') + 1
        
        # Get current scroll position
        scroll_pos = editor.yview()[0]
        
        # Enable editing of line numbers
        line_numbers.config(state='normal')
        line_numbers.delete('1.0', tk.END)
        
        # Add line numbers
        line_number_text = '\n'.join(str(i).rjust(3) for i in range(1, line_count + 1))
        line_numbers.insert('1.0', line_number_text)
        
        # Disable editing of line numbers
        line_numbers.config(state='disabled')
        
        # Restore scroll position
        line_numbers.yview_moveto(scroll_pos)
        
        # Reset modified flag
        editor.edit_modified(False)

    def open_file(self, path):
        """Open file in a new tab or switch to existing tab"""
        try:
            # If file is already open, just switch to its tab
            if path in self.tabs:
                self.switch_tab(path)
                return
                
            # Read file contents
            with open(path, 'r', encoding='utf-8') as file:
                content = file.read()
            
            # Create new tab
            tab = self.create_tab(path)
            
            # Insert content
            tab['editor'].delete('1.0', tk.END)
            tab['editor'].insert('1.0', content)
            
            # Force update of line numbers
            self.update_line_numbers(tab['editor'], tab['line_numbers'])
            
            # Switch to new tab
            self.switch_tab(path)
            
            # Store file information
            self.current_file = path
            self.file_contents[path] = content

            if path.lower().endswith('.urdf'):
                if self.simulator:
                    self.simulator.load_urdf(path)
                else:
                    # Start simulation if not running
                    self.start_simulation()
                    # Give time for simulator to initialize
                    self.root.after(1000, lambda: self.simulator.load_urdf(path) if self.simulator else None)
        except Exception as e:
            print(f"Error opening file: {str(e)}")
            tk.messagebox.showerror("Error", f"Could not open file: {str(e)}")

    def save_file(self, editor):
        """Save the current file"""
        if self.active_tab:
            try:
                content = editor.get("1.0", tk.END)
                with open(self.active_tab, 'w', encoding='utf-8') as file:
                    file.write(content)
                # Update stored content
                self.file_contents[self.active_tab] = content
                print(f"Saved: {self.active_tab}")  # Optional feedback
            except Exception as e:
                print(f"Error saving file: {str(e)}")
                tk.messagebox.showerror("Error", f"Could not save file: {str(e)}")

    def update_tab_label(self, path):
        # Get file name from path
        file_name = os.path.basename(path)
        
        # Find and update tab label
        for widget in self.editor_frame.winfo_children():
            if isinstance(widget, tk.Frame):  # Tab frame
                for child in widget.winfo_children():
                    if isinstance(child, tk.Label):  # Tab label
                        child.config(text=file_name)
                        break
                break

    def get_launch_files(self):
        """Get list of launch files from the ROS2 workspace"""
        launch_files = []
        self.launch_file_paths.clear() 
        
        if hasattr(self, 'project_root'):
            # Look for src directory
            src_dir = os.path.join(self.project_root, 'src')
            if os.path.exists(src_dir):
                for root, dirs, files in os.walk(src_dir):
                    if 'launch' in os.path.basename(root):
                        for file in files:
                            if file.endswith('.launch.py'):
                                # Get the full path
                                full_path = os.path.join(root, file)
                                
                                # Get path relative to src directory
                                rel_path = os.path.relpath(full_path, src_dir)
                                
                                # Store the mapping
                                self.launch_file_paths[rel_path] = full_path
                                
                                # Add the src-relative path to the list
                                launch_files.append(rel_path)
        
        # Sort the launch files for better organization
        launch_files.sort()
        
        # If no launch files found, add a placeholder
        if not launch_files:
            launch_files = ["No launch files found"]
            
        return launch_files

    def on_launch_file_selected(self, event):
        """Handle launch file selection from combobox"""
        selected_file = self.run_combo.get()
        if selected_file and selected_file != "No launch files found":
            # Get the full path from our mapping
            full_path = self.launch_file_paths.get(selected_file)
            if full_path and os.path.exists(full_path):
                self.open_file(full_path)

    def create_terminal(self):
        """Create an embedded terminal at the bottom of the IDE"""
        # Create bottom section for terminal
        self.terminal_frame = ttk.Frame(self.root)
        self.terminal_frame.grid(row=2, column=1, sticky="nsew")
        
        # Configure terminal header
        header = ttk.Frame(self.terminal_frame)
        header.pack(fill=tk.X, padx=2, pady=(2,0))
        
        self.terminal_label = ttk.Label(header, text="Terminal")
        self.terminal_label.pack(side=tk.LEFT, padx=5)
        
        # Create close button for terminal
        close_btn = ttk.Button(header, text="×", width=3,
                             command=self.toggle_terminal)
        close_btn.pack(side=tk.RIGHT, padx=2)
        
        # Create terminal text widget
        self.terminal = tk.Text(self.terminal_frame, 
                              bg=self.bg_darker,
                              fg="#00FF00",  # Green text
                              font=("Courier", 10),
                              height=10)
        self.terminal.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
        
        # Add scrollbar
        scrollbar = ttk.Scrollbar(self.terminal_frame, orient="vertical", 
                                command=self.terminal.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.terminal.config(yscrollcommand=scrollbar.set)
        
        # Initialize terminal process variables
        self.terminal_queue = queue.Queue()
        self.terminal_running = False
        
        # Hide terminal initially
        self.terminal_frame.grid_remove()
        
    def toggle_terminal(self):
        """Show/hide the terminal"""
        if self.terminal_frame.winfo_viewable():
            self.terminal_frame.grid_remove()
        else:
            self.terminal_frame.grid()
            # Adjust grid weights
            self.root.grid_rowconfigure(1, weight=3)  # Main content
            self.root.grid_rowconfigure(2, weight=1)  # Terminal

    def write_to_terminal(self, text):
        """Write text to terminal and scroll to bottom"""
        self.terminal.insert(tk.END, text)
        self.terminal.see(tk.END)
        self.terminal.update_idletasks()

    def toggle_run_state(self):
        """Toggle between run and stop states"""
        if not self.is_running:
            self.start_ros2_launch()
        else:
            self.stop_ros2_launch()
    
    def update_run_button(self, running=False):
        """Update run button appearance and state"""
        if running:
            self.run_btn.configure(text="⬛")  # Stop symbol
            self.run_btn_frame.configure(bg=self.accent_color)  # Highlight when running
            self.is_running = True
        else:
            self.run_btn.configure(text="▶")  # Play symbol
            self.run_btn_frame.configure(bg=self.bg_darker)  # Normal background
            self.is_running = False
            
    def start_ros2_launch(self):
        """Start ROS2 launch with updated button state and PyBullet integration"""
        selected_file = self.run_combo.get()
        if not selected_file or selected_file == "No launch files found":
            return

        # Show terminal if hidden
        if not self.terminal_frame.winfo_viewable():
            self.toggle_terminal()
        
        # Clear terminal
        self.terminal.delete(1.0, tk.END)
        
        # Start ROS node if not already started
        # if self.ros_node is None:
        #     try:
        #         rclpy.init()
        #         self.ros_node = rclpy.create_node('ide_node')
                
        #         # Create robot loader
        #         self.robot_loader = RobotLoader()
                
        #         # Start ROS spin thread
        #         def ros_spin():
        #             while rclpy.ok():
        #                 rclpy.spin_once(self.robot_loader, timeout_sec=0.1)
                
        #         self.ros_spin_thread = threading.Thread(target=ros_spin, daemon=True)
        #         self.ros_spin_thread.start()
                
        #     except Exception as e:
        #         self.write_to_terminal(f"Error initializing ROS2: {str(e)}\n")
        #         return
        
        # Initialize PyBullet bridge
        # if not hasattr(self, 'pybullet_bridge'):
        #     self.pybullet_bridge = ROSPyBulletBridge(800, 600)
        
        # Start PyBullet simulator
        # self.pybullet_bridge.start()
        
        # Monitor robot description queue
        # def check_robot_description():
        #     try:
        #         # Check for new robot description
        #         description = self.robot_loader.description_queue.get_nowait()
        #         self.pybullet_bridge.load_robot_from_description(description)
        #     except queue.Empty:
        #         if self.is_running:
        #             self.root.after(100, check_robot_description)
                    
        # self.root.after(100, check_robot_description)
        
        # Get the full path of the launch file
        launch_file = self.launch_file_paths.get(selected_file)
        if not launch_file or not os.path.exists(launch_file):
            self.write_to_terminal("Error: Launch file not found\n")
            return
            
        # Update button state before launching
        self.update_run_button(running=True)
        
        # Create and run the launch script
        self.run_in_terminal([
            "bash", "-c",
            f"""
            # Source ROS2 Iron
            source /opt/ros/iron/setup.bash
            
            # Source project setup
            source {os.path.join(self.project_root, 'install/setup.bash')}
            
            # Launch the file
            ros2 launch {launch_file}
            """
        ])

    def stop_ros2_launch(self):
        """Stop all running ROS2 processes and cleanup"""
        try:
            # Stop PyBullet simulator
            if hasattr(self, 'pybullet_bridge'):
                self.pybullet_bridge.stop()
            
            # Shutdown ROS node
            if self.ros_node:
                self.ros_node.destroy_node()
                rclpy.shutdown()
                self.ros_node = None
                # self.robot_loader = None
            
            if self.process_pid:
                # Get the parent process
                parent = psutil.Process(self.process_pid)
                
                # Get all child processes
                children = parent.children(recursive=True)
                
                # Stop terminal output reading
                self.terminal_running = False
                
                # Terminate all child processes
                for child in children:
                    try:
                        child.terminate()
                    except psutil.NoSuchProcess:
                        continue
                
                # Terminate parent process
                parent.terminate()
                
                # Wait for processes to terminate
                gone, alive = psutil.wait_procs(children + [parent], timeout=3)
                
                # Force kill any remaining processes
                for p in alive:
                    try:
                        p.kill()
                    except psutil.NoSuchProcess:
                        continue
                
                self.write_to_terminal("\nProcesses terminated.\n")
                
            # Reset process tracking
            self.process_pid = None
            self.current_process = None
            
            # Update button state
            self.update_run_button(running=False)
            
        except Exception as e:
            self.write_to_terminal(f"\nError stopping processes: {str(e)}\n")
            # Ensure button is reset even if error occurs
            self.update_run_button(running=False)

    def run_in_terminal(self, command):
        """Run a command in the terminal with real-time output"""
        def read_output(master_fd):
            while self.terminal_running:
                try:
                    r, _, _ = select.select([master_fd], [], [], 0.1)
                    if master_fd in r:
                        output = os.read(master_fd, 1024).decode()
                        self.terminal_queue.put(output)
                        self.root.event_generate('<<TerminalUpdate>>')
                except (OSError, IOError):
                    break

        def update_terminal():
            try:
                while True:
                    output = self.terminal_queue.get_nowait()
                    self.write_to_terminal(output)
            except queue.Empty:
                if self.terminal_running:
                    self.root.after(100, update_terminal)

        try:
            # Create pseudo-terminal
            master_fd, slave_fd = pty.openpty()
            
            # Set terminal size
            rows, cols = 24, 80
            size = struct.pack("HHHH", rows, cols, 0, 0)
            fcntl.ioctl(slave_fd, termios.TIOCSWINSZ, size)
            
            # Start the process
            process = subprocess.Popen(
                command,
                stdin=slave_fd,
                stdout=slave_fd,
                stderr=slave_fd,
                preexec_fn=os.setsid,
                shell=False
            )
            
            # Store process information
            self.current_process = process
            self.process_pid = process.pid
            
            # Start output reading thread
            self.terminal_running = True
            threading.Thread(target=read_output, args=(master_fd,), daemon=True).start()
            
            # Start terminal update
            self.root.after(100, update_terminal)
            
            # Monitor process completion
            def check_process():
                if process.poll() is not None:
                    self.terminal_running = False
                    self.update_run_button(running=False)
                    os.close(master_fd)
                    os.close(slave_fd)
                elif self.is_running:
                    self.root.after(100, check_process)
            
            self.root.after(100, check_process)
            
        except Exception as e:
            self.write_to_terminal(f"Error: {str(e)}\n")
            self.update_run_button(running=False)

if __name__ == "__main__":
    root = tk.Tk()
    root.geometry("1200x800")
    root.attributes("-zoomed", True)
    app = IDEApp(root)
    # Bind window resize event to update proportions
    def on_resize(event):
        if event.widget == root:
            app.set_paned_proportions()
    
    root.bind("<Configure>", on_resize)
    root.mainloop()