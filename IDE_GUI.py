import tkinter as tk
from tkinter import ttk
import tkinter.font as tkfont

class IDEApp:
    def __init__(self, root):
        self.root = root
        self.root.title("MyProject")
        
        # Configure colors
        self.bg_dark = "#1e1e1e"
        self.bg_darker = "#252526"
        self.bg_sidebar = "#333333"
        self.bg_lighter = "#2d2d2d"
        self.text_color = "#d4d4d4"
        self.text_color_dim = "#858585"
        self.accent_color = "#3c8039"  # Green for preview
        self.select_color = "#1f538d"  # Blue for selection
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
        
        # Create icon sidebar
        # self.create_icon_sidebar()
        
        # Create toolbar
        self.create_toolbar()
        
        # Create main container
        self.main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.main_paned.grid(row=1, column=1, sticky="nsew", padx=1, pady=(1, 1))
        
        # Create file explorer
        self.create_file_explorer()
        
        # Create editor and preview paned window
        self.editor_preview_paned = ttk.PanedWindow(self.main_paned, orient=tk.HORIZONTAL)
        self.main_paned.add(self.editor_preview_paned)
        
        # Create code editor
        self.create_code_editor()
        
        # Create preview
        self.create_preview()
        
        # Set initial proportions
        self.root.update()
        self.set_paned_proportions()
        
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

        # Project icon using Canvas
        icon_size = 40
        icon_canvas = tk.Canvas(left_tools, 
                              width=icon_size, 
                              height=icon_size,
                              bg=self.bg_darker,
                              highlightthickness=0)
        icon_canvas.pack(side=tk.LEFT, padx=5)

        # Draw rounded rectangle
        radius = 2
        icon_canvas.create_rounded_rectangle = lambda x1, y1, x2, y2: icon_canvas.create_polygon(
            x1+radius, y1,
            x2-radius, y1,
            x2, y1+radius,
            x2, y2-radius,
            x2-radius, y2,
            x1+radius, y2,
            x1, y2-radius,
            x1, y1+radius,
            smooth=True,
            fill=self.button_color
        )
        
        # Create the rounded rectangle button
        icon_canvas.create_rounded_rectangle(2, 2, icon_size-2, icon_size-2)
        
        # Add text
        icon_canvas.create_text(icon_size/2, icon_size/2, 
                              text="MP", 
                              fill=self.text_color,
                              font=('Arial', 10, 'bold'))

        # Project dropdown
        # project_menu = tk.Menu(toolbar, tearoff=0, bg=self.bg_darker, fg=self.text_color,
        #                      activebackground=self.bg_lighter, activeforeground=self.text_color)
        # project_menu.add_command(label="New Project...")
        # project_menu.add_command(label="Open Project...")
        # project_menu.add_separator()
        # project_menu.add_command(label="Close Project")
        
        # project_btn = ttk.Menubutton(left_tools, text="MyProject", style="Toolbar.TMenubutton")
        # project_btn['menu'] = project_menu
        # project_btn.pack(side=tk.LEFT, padx=(2, 2))
        # Configure combobox style
        self.style.configure("Toolbar.TCombobox",
                            fieldbackground=self.bg_darker,
                            background=self.bg_darker,
                            foreground=self.text_color,
                            arrowcolor=self.text_color,
                            selectbackground=self.bg_lighter,
                            selectforeground=self.text_color)
        
        # Project combobox
        project_combo = ttk.Combobox(left_tools, values=["New Project...", "Open Project...", "Close Project"],
                                    style="Toolbar.TCombobox", width=15)
        project_combo.set("MyProject")
        project_combo.pack(side=tk.LEFT, padx=(2, 10))

        # Version Control combobox
        vc_combo = ttk.Combobox(left_tools, values=["Git Operations", "Commit", "Push", "Pull"],
                            style="Toolbar.TCombobox", width=20)
        vc_combo.set("Version Control")
        vc_combo.pack(side=tk.LEFT, padx=2)

         # Right side of toolbar
        right_tools = tk.Frame(toolbar, bg=self.bg_darker)
        right_tools.grid(row=0, column=2, sticky="e")
        
        # Raspberry Pi combobox
        pi_combo = ttk.Combobox(right_tools, values=["Connect", "Disconnect", "Settings"],
                            style="Toolbar.TCombobox", width=15)
        pi_combo.set("🤖 Raspberry Pi")
        pi_combo.pack(side=tk.LEFT, padx=2)
        
        # Build button
        build_btn = ttk.Button(right_tools, text="Build", style="Toolbar.TButton")
        build_btn.pack(side=tk.LEFT, padx=2)
        
        # Run combobox
        run_combo = ttk.Combobox(right_tools, values=["Run Project", "Debug", "Run with Arguments"],
                                style="Toolbar.TCombobox", width=10)
        run_combo.set("Run")
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
        
    def create_file_explorer(self):
        # Sidebar frame
        self.sidebar_frame = tk.Frame(self.main_paned, bg=self.bg_darker)
        self.main_paned.add(self.sidebar_frame)
        
        # Project label
        project_label = tk.Label(self.sidebar_frame, text="MyProject",
                               bg=self.bg_darker, fg=self.text_color,
                               font=('Arial', 11, 'bold'), anchor='w')
        project_label.pack(fill=tk.X, padx=10, pady=(5,5))
        
        # Treeview for file explorer
        self.tree = ttk.Treeview(self.sidebar_frame, show='tree')
        self.tree.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Populate the tree
        self.populate_tree()
        
    def create_code_editor(self):
        # Editor frame
        self.editor_frame = tk.Frame(self.editor_preview_paned, bg=self.bg_dark)
        self.editor_preview_paned.add(self.editor_frame)
        
        # Tab header
        tab_frame = tk.Frame(self.editor_frame, bg=self.bg_darker)
        tab_frame.pack(fill=tk.X)
        
        tab_label = tk.Label(tab_frame, text="main.xml", bg=self.bg_darker,
                           fg=self.text_color_dim, padx=10, pady=5)
        tab_label.pack(side=tk.LEFT)
        
        # Editor container
        editor_container = tk.Frame(self.editor_frame, bg=self.bg_dark)
        editor_container.pack(fill=tk.BOTH, expand=True)
        
        # Line numbers
        self.line_numbers = tk.Text(editor_container, width=6, padx=5, pady=5,
                                  bg=self.bg_dark, fg=self.text_color_dim,
                                  bd=0, highlightthickness=0)
        self.line_numbers.pack(side=tk.LEFT, fill=tk.Y)
        
        # Generate line numbers
        lines = '\n'.join(str(i).rjust(3) for i in range(1, 101))
        self.line_numbers.insert("1.0", lines)
        self.line_numbers.config(state='disabled')
        
        # Main text editor with monospace font
        self.editor = tk.Text(editor_container, wrap=tk.NONE, padx=5, pady=5,
                            bg=self.bg_dark, fg=self.text_color,
                            insertbackground=self.text_color,
                            bd=0, highlightthickness=0,
                            font=('Courier New', 10))
        self.editor.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Scrollbars
        v_scrollbar = ttk.Scrollbar(editor_container, orient="vertical",
                                  command=self.editor.yview)
        v_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        h_scrollbar = ttk.Scrollbar(self.editor_frame, orient="horizontal",
                                  command=self.editor.xview)
        h_scrollbar.pack(side=tk.BOTTOM, fill=tk.X)
        
        self.editor.config(yscrollcommand=v_scrollbar.set,
                         xscrollcommand=h_scrollbar.set)
        
        # Sync line numbers with editor scroll
        self.editor.config(yscrollcommand=lambda *args: self.sync_scroll(v_scrollbar, args))
        
        # Insert sample code
        self.insert_sample_code()
        
    def sync_scroll(self, scrollbar, args):
        # Sync line numbers with editor scroll
        self.line_numbers.yview_moveto(args[0])
        scrollbar.set(*args)
        
    def create_preview(self):
        # Preview frame
        preview_frame = tk.Frame(self.editor_preview_paned, bg=self.accent_color)
        self.editor_preview_paned.add(preview_frame)
        
        # Preview header
        header_frame = tk.Frame(preview_frame, bg=self.bg_darker)
        header_frame.pack(fill=tk.X)
        
        preview_label = tk.Label(header_frame, text="Simulator Preview",
                               bg=self.bg_darker, fg=self.text_color,
                               padx=10, pady=5)
        preview_label.pack(side=tk.LEFT)
        
    def populate_tree(self):
        project = self.tree.insert("", "end", text="MyProject", open=True)
        modules = self.tree.insert(project, "end", text="modules", open=True)
        urdf = self.tree.insert(modules, "end", text="urdf", open=True)
        self.tree.insert(urdf, "end", text="main.xml")
        
        # Add folders and files
        actuators = self.tree.insert(urdf, "end", text="actuators", open=True)
        self.tree.insert(actuators, "end", text="drivers.py")
        self.tree.insert(actuators, "end", text="controllers.py")
        self.tree.insert(actuators, "end", text="run.py")
        
        sensors = self.tree.insert(urdf, "end", text="sensors", open=True)
        self.tree.insert(sensors, "end", text="drivers.py")
        self.tree.insert(sensors, "end", text="run.py")
        
        world = self.tree.insert(urdf, "end", text="world", open=True)
        self.tree.insert(world, "end", text="world.py")
        
        comms = self.tree.insert(urdf, "end", text="comms", open=True)
        self.tree.insert(comms, "end", text="comms.py")
        
        self.tree.insert(urdf, "end", text="main.py")
        self.tree.insert(urdf, "end", text="package.xml")
        
    def insert_sample_code(self):
        sample_code = '''<!-- CHASSIS LINK -->
<joint name="chassis_joint" type="fixed">
    <parent link="base_link"/>
    <child link="chassis"/>
    <origin xyz="${-wheel_offset_x} 0 ${-wheel_offset_z}"/>
</joint>
<link name="chassis">
    <visual>
        <origin xyz="${chassis_length/2} 0 ${chassis_height/2}"/>
        <geometry>
            <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
        </geometry>
        <material name="orange"/>
    </visual>
    <collision>
        <origin xyz="${chassis_length/2} 0 ${chassis_height/2}"/>
        <geometry>
            <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
        </geometry>
    </collision>
    <xacro:inertial_box mass="0.5" x="${chassis_length}"
y="${chassis_width}" z="${chassis_height}">
        <origin xyz="${chassis_length/2} 0 ${chassis_height/2}"
rpy="0 0 0"/>
    </xacro:inertial_box>
</link>'''
        self.editor.insert("1.0", sample_code)

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