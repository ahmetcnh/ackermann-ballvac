#!/usr/bin/env python3
"""
Robot State Monitor GUI

Displays real-time state information for all robots in the multi-robot
ball collection system, including:
- Current robot state (IDLE, EXPLORING, NAVIGATING, APPROACHING, COLLECTING, RECOVERING)
- Collected ball colors
- Target ball information

Usage:
    ros2 run ballvac_ball_collector robot_state_monitor.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from ballvac_msgs.msg import RobotStatus

import tkinter as tk
from tkinter import ttk
import threading
from datetime import datetime


# Robot state names mapping
STATE_NAMES = {
    0: 'IDLE',
    1: 'NAVIGATING', 
    2: 'APPROACHING',
    3: 'COLLECTING',
    4: 'STUCK',
    5: 'EXPLORING',
}

# Color RGB values for display
BALL_COLORS = {
    'red': '#FF0000',
    'green': '#00FF00',
    'blue': '#0000FF',
    'yellow': '#FFFF00',
    'cyan': '#00FFFF',
    'purple': '#9B00FF',
    'orange': '#FF8000',
    'pink': '#FF0080',
    'lime': '#80FF00',
    'teal': '#008080',
}

# Robot display colors
ROBOT_COLORS = {
    'ballvac1': '#FF4444',  # Red
    'ballvac2': '#4444FF',  # Blue
    'ballvac3': '#44FF44',  # Green
}


class RobotStateMonitor(Node):
    """ROS2 node for monitoring robot states."""
    
    def __init__(self, gui_callback):
        super().__init__('robot_state_monitor')
        
        self.gui_callback = gui_callback
        self.robot_states = {
            'ballvac1': {'state': 'IDLE', 'target': '', 'collected': []},
            'ballvac2': {'state': 'IDLE', 'target': '', 'collected': []},
            'ballvac3': {'state': 'IDLE', 'target': '', 'collected': []},
        }
        
        # QoS for status messages
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribe to robot status
        self.status_sub = self.create_subscription(
            RobotStatus,
            '/fleet/robot_status',
            self.status_callback,
            qos
        )
        
        # Subscribe to ball deletion events
        self.deleted_sub = self.create_subscription(
            String,
            '/fleet/ball_deleted',
            self.ball_deleted_callback,
            10
        )
        
        # Subscribe to completion message from ball_launcher
        self.completion_sub = self.create_subscription(
            String,
            '/ball_launcher_node/launch_info',
            self.completion_callback,
            10
        )
        
        # Track completion
        self.completion_time = None
        
        # Update timer
        self.create_timer(0.5, self.update_gui)
        
        self.get_logger().info('Robot State Monitor started!')
    
    def status_callback(self, msg: RobotStatus):
        """Handle robot status messages."""
        robot_id = msg.robot_id
        if robot_id not in self.robot_states:
            return
        
        # Update state
        state_name = STATE_NAMES.get(msg.state, 'UNKNOWN')
        self.robot_states[robot_id]['state'] = state_name
        
        # Update target ball (use target_ball_id field)
        if msg.target_ball_id:
            self.robot_states[robot_id]['target'] = msg.target_ball_id
        elif msg.assigned_ball_id:
            self.robot_states[robot_id]['target'] = msg.assigned_ball_id.replace('ball_', '')
        else:
            self.robot_states[robot_id]['target'] = ''
        
        # Update collected balls from collected_ball_colors array
        if msg.collected_ball_colors:
            self.robot_states[robot_id]['collected'] = list(msg.collected_ball_colors)
    
    def ball_deleted_callback(self, msg: String):
        """Handle ball deletion events to track collections."""
        data = msg.data
        if ':' in data:
            ball_name, robot_id = data.split(':', 1)
        else:
            ball_name = data
            robot_id = None
        
        color = ball_name.replace('ball_', '')
        
        if robot_id and robot_id in self.robot_states:
            if color not in self.robot_states[robot_id]['collected']:
                self.robot_states[robot_id]['collected'].append(color)
    
    def completion_callback(self, msg: String):
        """Handle completion message from ball_launcher."""
        data = msg.data
        if data.startswith('COMPLETE:'):
            time_str = data.replace('COMPLETE:', '')
            self.completion_time = time_str
            self.get_logger().info(f'All balls collected! Total time: {time_str}')
    
    def update_gui(self):
        """Send state update to GUI."""
        self.gui_callback(self.robot_states.copy(), self.completion_time)


class RobotStateGUI:
    """Tkinter GUI for displaying robot states."""
    
    def __init__(self):
        self.root = tk.Tk()
        self.root.title('Robot State Monitor')
        self.root.geometry('650x450')
        self.root.configure(bg='#2D2D2D')
        
        # Title
        title = tk.Label(
            self.root,
            text='🤖 Multi-Robot Ball Collection Monitor',
            font=('Helvetica', 16, 'bold'),
            bg='#2D2D2D',
            fg='#FFFFFF'
        )
        title.pack(pady=10)
        
        # Robot frames container
        self.robot_container = tk.Frame(self.root, bg='#2D2D2D')
        self.robot_container.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Create robot cards
        self.robot_cards = {}
        for i, robot_name in enumerate(['ballvac1', 'ballvac2', 'ballvac3']):
            card = self.create_robot_card(robot_name, i)
            self.robot_cards[robot_name] = card
        
        # Status bar
        self.status_bar = tk.Label(
            self.root,
            text='Connecting to ROS2...',
            font=('Helvetica', 10),
            bg='#1D1D1D',
            fg='#888888',
            anchor='w'
        )
        self.status_bar.pack(fill='x', side='bottom')
    
    def create_robot_card(self, robot_name: str, index: int) -> dict:
        """Create a card widget for a robot."""
        color = ROBOT_COLORS.get(robot_name, '#888888')
        
        frame = tk.Frame(
            self.robot_container,
            bg='#3D3D3D',
            relief='raised',
            bd=2
        )
        frame.grid(row=0, column=index, padx=5, pady=5, sticky='nsew')
        self.robot_container.columnconfigure(index, weight=1)
        self.robot_container.rowconfigure(0, weight=1)
        
        # Robot name with color indicator
        header = tk.Frame(frame, bg=color)
        header.pack(fill='x')
        
        name_label = tk.Label(
            header,
            text=f'  {robot_name.upper()}',
            font=('Helvetica', 14, 'bold'),
            bg=color,
            fg='#FFFFFF',
            anchor='w'
        )
        name_label.pack(fill='x', pady=5)
        
        # State label
        state_frame = tk.Frame(frame, bg='#3D3D3D')
        state_frame.pack(fill='x', padx=10, pady=5)
        
        tk.Label(
            state_frame,
            text='State:',
            font=('Helvetica', 10),
            bg='#3D3D3D',
            fg='#AAAAAA'
        ).pack(side='left')
        
        state_label = tk.Label(
            state_frame,
            text='IDLE',
            font=('Helvetica', 12, 'bold'),
            bg='#3D3D3D',
            fg='#FFFF00'
        )
        state_label.pack(side='left', padx=5)
        
        # Target label
        target_frame = tk.Frame(frame, bg='#3D3D3D')
        target_frame.pack(fill='x', padx=10, pady=5)
        
        tk.Label(
            target_frame,
            text='Target:',
            font=('Helvetica', 10),
            bg='#3D3D3D',
            fg='#AAAAAA'
        ).pack(side='left')
        
        target_label = tk.Label(
            target_frame,
            text='None',
            font=('Helvetica', 10),
            bg='#3D3D3D',
            fg='#FFFFFF'
        )
        target_label.pack(side='left', padx=5)
        
        # Collected balls section
        collected_frame = tk.Frame(frame, bg='#3D3D3D')
        collected_frame.pack(fill='x', padx=10, pady=10)
        
        tk.Label(
            collected_frame,
            text='Collected:',
            font=('Helvetica', 10),
            bg='#3D3D3D',
            fg='#AAAAAA'
        ).pack(anchor='w')
        
        balls_container = tk.Frame(collected_frame, bg='#3D3D3D')
        balls_container.pack(fill='x', pady=5)
        
        return {
            'frame': frame,
            'state_label': state_label,
            'target_label': target_label,
            'balls_container': balls_container,
            'ball_widgets': []
        }
    
    def update_robot_state(self, robot_name: str, state: str, target: str, collected: list):
        """Update the display for a robot."""
        if robot_name not in self.robot_cards:
            return
        
        card = self.robot_cards[robot_name]
        
        # Update state with color coding
        state_colors = {
            'IDLE': '#888888',
            'EXPLORING': '#00FFFF',
            'NAVIGATING': '#FFFF00',
            'APPROACHING': '#FF8800',
            'COLLECTING': '#00FF00',
            'STUCK': '#FF0000',
            'RECOVERING': '#FF0000',
        }
        card['state_label'].config(
            text=state,
            fg=state_colors.get(state, '#FFFFFF')
        )
        
        # Update target
        card['target_label'].config(text=target if target else 'None')
        
        # Update collected balls
        current_count = len(card['ball_widgets'])
        for i, color in enumerate(collected):
            if i >= current_count:
                ball_color = BALL_COLORS.get(color, '#888888')
                ball = tk.Canvas(
                    card['balls_container'],
                    width=20,
                    height=20,
                    bg='#3D3D3D',
                    highlightthickness=0
                )
                ball.create_oval(2, 2, 18, 18, fill=ball_color, outline='#FFFFFF')
                ball.pack(side='left', padx=2)
                card['ball_widgets'].append(ball)
    
    def update_status_bar(self, text: str):
        """Update the status bar."""
        self.status_bar.config(text=text)
    
    def run(self):
        """Start the GUI event loop."""
        self.root.mainloop()


def gui_update_callback(gui: RobotStateGUI, states: dict, completion_time=None):
    """Thread-safe GUI update."""
    def update():
        for robot_name, data in states.items():
            gui.update_robot_state(
                robot_name,
                data['state'],
                data['target'],
                data['collected']
            )
        
        if completion_time:
            gui.update_status_bar(f'🎉 ALL BALLS COLLECTED! Total time: {completion_time}')
        else:
            timestamp = datetime.now().strftime('%H:%M:%S')
            gui.update_status_bar(f'Last update: {timestamp}')
    
    gui.root.after(0, update)


def main():
    """Main entry point."""
    # Create GUI
    gui = RobotStateGUI()
    
    # Initialize ROS2
    rclpy.init()
    
    # Create ROS2 node with GUI callback
    def callback(states, completion_time=None):
        gui_update_callback(gui, states, completion_time)
    
    node = RobotStateMonitor(callback)
    
    # Run ROS2 spinner in background thread
    def ros_spin():
        rclpy.spin(node)
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    # Run GUI in main thread
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
