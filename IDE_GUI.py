import tkinter as tk
from tkinter import ttk
import tkinter.font as tkfont
import pybullet as p
import pybullet_data
import time
import threading
import os
import OpenGL
import ctypes
from PIL import Image, ImageTk
import numpy as np
import math
from tkinter import filedialog

class PhysicsSimulator:
    def __init__(self, canvas_width=800, canvas_height=600):
        if canvas_width <= 1 or canvas_height <= 1:
            canvas_width = 800
            canvas_height = 600

        self.width = canvas_width
        self.height = canvas_height
        self.physics_client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        self.cube_id = p.loadURDF("cube.urdf", [0, 0, 1])
        p.changeVisualShape(self.cube_id, -1, rgbaColor=[0, 1, 0, 1])
        
        # Camera parameters
        self.distance = 5
        self.yaw = 45
        self.pitch = 45
        self.target = [0, 0, 0]
        self.up_vector = [0, 0, 1]
        self.update_camera()
        
        self.running = True

    def update_camera(self):
        pos = [
            self.target[0] + self.distance * math.cos(math.radians(self.yaw)) * math.cos(math.radians(self.pitch)),
            self.target[1] + self.distance * math.sin(math.radians(self.yaw)) * math.cos(math.radians(self.pitch)),
            self.target[2] + self.distance * math.sin(math.radians(self.pitch))
        ]
        self.view_matrix = p.computeViewMatrix(pos, self.target, self.up_vector)
        self.proj_matrix = p.computeProjectionMatrixFOV(60, float(self.width)/self.height, 0.1, 100.0)

    def zoom(self, factor):
        self.distance = max(0.1, min(20, self.distance * factor))
        self.update_camera()

    def rotate(self, dyaw, dpitch):
        self.yaw += dyaw
        self.pitch = max(-89, min(89, self.pitch + dpitch))
        self.update_camera()

    def get_frame(self):
        # Get rendering from PyBullet
        width, height, rgba, depth, seg = p.getCameraImage(
            width=self.width,
            height=self.height,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.proj_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        
        # Convert to PIL Image
        rgba_array = np.reshape(rgba, (height, width, 4))
        rgb_array = rgba_array[:, :, :3]
        return Image.fromarray(rgb_array)

    def step(self):
        while self.running:
            p.stepSimulation()
            time.sleep(1/240)

    def stop(self):
        self.running = False
        p.disconnect()


class IDEApp:
    def __init__(self, root):
        # Initialize root window and basic properties
        self.root = root
        self.root.title("MyProject")
        self.folder_name = "MyProject"
        
        # Initialize file tracking
        self.current_file = None
        self.file_contents = {}
        
        # Initialize simulation properties
        self.simulator = None
        self.sim_thread = None
        self.render_thread = None
        
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
        
        # Create file explorer first
        self.create_file_explorer()
        
        # Create editor and preview paned window
        self.editor_preview_paned = ttk.PanedWindow(self.main_paned, orient=tk.HORIZONTAL)
        self.main_paned.add(self.editor_preview_paned)
        
        # Create remaining UI elements
        self.create_code_editor()
        self.create_simulator_preview()
        
        # Set initial proportions
        self.root.update()
        self.set_paned_proportions()

    def __del__(self):
        self.stop_simulation()
        
    def set_paned_proportions(self):
        # Get total width (excluding icon sidebar)
        total_width = self.root.winfo_width() - 50  # 50 is icon sidebar width
        
        # Calculate positions for main paned window
        explorer_width = int(total_width * 0.25)
        self.main_paned.sashpos(0, explorer_width)
        
        # Calculate positions for editor/preview paned window
        remaining_width = total_width - explorer_width
        editor_width = int(remaining_width * 0.60)  # Give editor 45% of remaining space
        self.editor_preview_paned.sashpos(0, editor_width)
        
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
        run_combo = ttk.Combobox(right_tools, values=["main.py"],
                                style="Toolbar.TCombobox", width=10)
        run_combo.set("main.py")
        run_combo.pack(side=tk.LEFT, padx=2)
        
        # Play button
        play_btn = ttk.Button(right_tools, text="▶", style="Toolbar.TButton")
        play_btn.pack(side=tk.LEFT, padx=2)
        
        # Icons
        search_btn = ttk.Button(right_tools, text="Search", style="Toolbar.TButton")
        settings_btn = ttk.Button(right_tools, text="Settings", style="Toolbar.TButton")
        user_btn = ttk.Button(right_tools, text="Account", style="Toolbar.TButton")
        
        search_btn.pack(side=tk.LEFT, padx=2)
        settings_btn.pack(side=tk.LEFT, padx=2)
        user_btn.pack(side=tk.LEFT, padx=2)

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
        self.main_paned.add(self.sidebar_frame)
        
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
        self.editor_preview_paned.add(self.editor_frame)
        
        # Tab container
        self.tab_container = tk.Frame(self.editor_frame, bg=self.bg_darker)
        self.tab_container.pack(fill=tk.X)
        
        # Dictionary to store tabs and their content
        self.tabs = {}  # {filepath: {'tab': tab_frame, 'editor': editor, 'label': label}}
        self.active_tab = None
        
        # Editor container for the actual editor widgets
        self.editor_container = tk.Frame(self.editor_frame, bg=self.bg_dark)
        self.editor_container.pack(fill=tk.BOTH, expand=True)

    def sync_scroll(self, scrollbar, args):
        # Sync line numbers with editor scroll
        self.line_numbers.yview_moveto(args[0])
        scrollbar.set(*args)

    def start_simulation(self):
        if self.simulator is None:
            self.simulator = PhysicsSimulator()
            self.sim_thread = threading.Thread(target=self.simulator.step)
            self.render_thread = threading.Thread(target=self.render_loop)
            self.sim_thread.start()
            self.render_thread.start()

    def render_loop(self):
        while self.simulator and self.simulator.running:
            frame = self.simulator.get_frame()
            photo = ImageTk.PhotoImage(frame)
            self.sim_canvas.create_image(0, 0, image=photo, anchor=tk.NW)
            self.sim_canvas.image = photo  # Keep reference
            time.sleep(1/60)  # 60 FPS

    def stop_simulation(self):
        if self.simulator:
            self.simulator.stop()
            self.sim_thread.join()
            self.render_thread.join()
            self.simulator = None

    def create_simulator_preview(self):
        preview_frame = tk.Frame(self.editor_preview_paned, bg=self.bg_darker)
        self.editor_preview_paned.add(preview_frame)
        preview_frame.pack_propagate(False)
        preview_frame.grid_rowconfigure(1, weight=1)
        preview_frame.grid_columnconfigure(0, weight=1)

        # Header setup remains the same
        header_frame = tk.Frame(preview_frame, bg=self.bg_darker)
        header_frame.grid(row=0, column=0, sticky="ew")

        preview_label = tk.Label(header_frame, text="Preview",
                            bg=self.bg_darker, fg=self.text_color,
                            padx=10, pady=5)
        preview_label.pack(side=tk.LEFT)

        controls_frame = tk.Frame(header_frame, bg=self.bg_darker)
        controls_frame.pack(side=tk.RIGHT, padx=5)

        self.start_btn = ttk.Button(controls_frame, text="Start Simulation", 
                            command=lambda: self.start_simulation())
        self.start_btn.pack(side=tk.LEFT, padx=2)

        stop_btn = ttk.Button(controls_frame, text="Stop Simulation", 
                            command=self.stop_simulation)
        stop_btn.pack(side=tk.LEFT, padx=2)

        self.sim_canvas = tk.Canvas(preview_frame, bg=self.bg_darker)
        self.sim_canvas.grid(row=1, column=0, sticky="nsew", padx=0, pady=0)

        # Mouse controls
        self.last_x = 0
        self.last_y = 0
        self.sim_canvas.bind("<Button-1>", self.mouse_down)
        self.sim_canvas.bind("<B1-Motion>", self.mouse_move)
        self.sim_canvas.bind('<Button-4>', self.mouse_wheel)  # Linux touchpad up
        self.sim_canvas.bind('<Button-5>', self.mouse_wheel)  # Linux touchpad down
        self.sim_canvas.bind('<MouseWheel>', self.mouse_wheel)  # Windows/macOS wheel
        # Track touchpad gestures
        self.sim_canvas.bind('<Control-Button-4>', self.touchpad_zoom)  # Pinch out
        self.sim_canvas.bind('<Control-Button-5>', self.touchpad_zoom)  # Pinch in
        
        
        def init_simulator():
            width = preview_frame.winfo_width()
            height = preview_frame.winfo_height() - header_frame.winfo_height()
            
            if width > 1 and height > 1:
                self.simulator = PhysicsSimulator(width, height)
                self.sim_thread = threading.Thread(target=self.simulator.step)
                self.render_thread = threading.Thread(target=self.render_loop)
                self.sim_thread.start()
                self.render_thread.start()

        preview_frame.update()
        self.start_simulation = init_simulator

        def update_canvas_size(event):
            if self.simulator:
                self.simulator.width = event.width
                self.simulator.height = event.height
                self.simulator.update_camera()
        
        self.sim_canvas.bind('<Configure>', update_canvas_size)
    
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
            
        item_id = selected_items[0]  # Get the first selected item
        
        # Get item info
        item = self.tree.item(item_id)
        item_path = item['values'][0] if item['values'] else None
        
        # If it's a file and we have a path, open it
        if item_path and os.path.isfile(item_path):
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