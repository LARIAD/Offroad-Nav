#https://docs.omniverse.nvidia.com/kit/docs/omni.graph/latest/omni.graph.core.Classes.html
#https://docs.omniverse.nvidia.com/kit/docs/omni.graph/latest/omni.graph.core/omni.graph.core.Graph.html
#https://docs.omniverse.nvidia.com/kit/docs/omni.graph/latest/omni.graph.core/omni.graph.core.Node.html
#https://docs.omniverse.nvidia.com/kit/docs/omni.graph/latest/omni.graph.core/omni.graph.core.Attribute.html#omni.graph.core.Attribute

from isaacsim import SimulationApp
import carb
import carb.settings
import time
import argparse
import rospy

# Parse --gui flag early (before SimulationApp init)
_pre_parser = argparse.ArgumentParser(add_help=False)
_pre_parser.add_argument('--gui', action='store_true', default=False)
_pre_args, _ = _pre_parser.parse_known_args()

# Start the application
simulation_app = SimulationApp({
    "headless": not _pre_args.gui,
    "renderer": "RayTracedLighting"  # or "PathTracing"
})

import omni.kit.app
manager = omni.kit.app.get_app().get_extension_manager()
settings = carb.settings.get_settings()

# Set the node name BEFORE enabling the extension
settings.set("/exts/isaacsim.ros1.bridge/nodeName", "OmniIsaacRosBridgeLidar")
manager.set_extension_enabled_immediate("isaacsim.ros1.bridge", True)

# Core imports after SimulationApp is initialized
from isaacsim.core.utils.stage import open_stage
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core import World
import omni.timeline
import omni.usd
import omni.graph.core as og  # Use Omnigraph directly

import rospy
from nav_msgs.msg import Odometry

# Plotting imports
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for saving files
import matplotlib.pyplot as plt

from datetime import datetime
import threading
import os
import signal
import sys


# Global flag for shutdown
shutdown_requested = False

def signal_handler(signum, frame):
    """Handle Ctrl+C signal."""
    global shutdown_requested
    print("\n[Signal] Ctrl+C received, saving trajectory and shutting down...")
    shutdown_requested = True

# Register signal handler
signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

def get_next_run_id(stage_name, output_dir):
    """Find the next available run ID by checking existing CSV files."""
    if not output_dir or not os.path.exists(output_dir):
        return 1
    
    existing_runs = []
    for f in os.listdir(output_dir):
        if f.startswith(stage_name + "_run") and f.endswith(".csv"):
            try:
                # Extract run number from filename like "stage_run001.csv"
                run_str = f.replace(stage_name + "_run", "").replace(".csv", "")
                existing_runs.append(int(run_str))
            except ValueError:
                pass
    
    if existing_runs:
        return max(existing_runs) + 1
    return 1


def load_all_runs(stage_name, output_dir):
    """Load all CSV files for a given stage."""
    runs = []
    
    if not output_dir or not os.path.exists(output_dir):
        return runs
    
    for f in sorted(os.listdir(output_dir)):
        if f.startswith(stage_name + "_run") and f.endswith(".csv"):
            filepath = os.path.join(output_dir, f)
            try:
                x, y = [], []
                with open(filepath, 'r') as file:
                    for line in file:
                        line = line.strip()
                        if line and not line.startswith('#'):
                            parts = line.split(',')
                            if len(parts) == 2:
                                x.append(float(parts[0]))
                                y.append(float(parts[1]))
                
                if x and y:
                    # Extract run ID from filename
                    run_str = f.replace(stage_name + "_run", "").replace(".csv", "")
                    run_id = int(run_str)
                    runs.append({'x': x, 'y': y, 'run_id': run_id, 'filename': f})
            except Exception as e:
                print(f"[Loader] Error loading {f}: {e}")
    
    return runs


class TrajectoryRecorder:
    """Class to record and save 2D trajectory."""
    
    def __init__(self, topic_name="/imu/odometry", output_dir=".", stage_name=""):
        self.topic_name = topic_name
        self.output_dir = output_dir
        self.stage_name = stage_name
        
        # Get next run ID
        self.run_id = get_next_run_id(stage_name, output_dir)
        
        # File paths
        self.csv_path = os.path.join(output_dir, f"{stage_name}_run{self.run_id:03d}.csv")
        self.pdf_path = os.path.join(output_dir, f"{stage_name}.pdf")
        
        self.pos_x = []
        self.pos_y = []
        
        self.lock = threading.Lock()
        self.subscriber = None
        self.message_count = 0
        
    def odometry_callback(self, msg):
        """Callback for odometry messages."""
        with self.lock:
            self.pos_x.append(msg.pose.pose.position.x)
            self.pos_y.append(msg.pose.pose.position.y)
            self.message_count += 1
            
            # if self.message_count % 100 == 0:
            #     print(f"[Recorder] Received {self.message_count} odometry messages")
    
    def start_subscriber(self):
        """Start the ROS subscriber."""
        if not rospy.core.is_initialized():
            rospy.init_node('isaac_sim_trajectory', anonymous=True, disable_signals=True)
        
        self.subscriber = rospy.Subscriber(
            self.topic_name, 
            Odometry, 
            self.odometry_callback,
            queue_size=10
        )
        print(f"Subscribed to {self.topic_name}")
        print(f"This is Run {self.run_id}")
    
    def save_csv(self):
        """Save current trajectory to CSV file."""
        with self.lock:
            if len(self.pos_x) < 2:
                print(f"[Recorder] Not enough data points: {len(self.pos_x)}")
                return False
            x = list(self.pos_x)
            y = list(self.pos_y)
        
        try:
            os.makedirs(self.output_dir, exist_ok=True)
            with open(self.csv_path, 'w') as f:
                f.write(f"# Run {self.run_id}\n")
                for xi, yi in zip(x, y):
                    f.write(f"{xi},{yi}\n")
            print(f"[Recorder] CSV saved: {self.csv_path}")
            return True
        except Exception as e:
            print(f"[Recorder] ERROR saving CSV: {e}")
            return False
    
    def save_pdf(self):
        """Generate PDF with all runs for this stage."""
        print(f"[Recorder] Generating PDF with all runs...")
        
        # Load all previous runs
        all_runs = load_all_runs(self.stage_name, self.output_dir)
        
        # Add current run
        # with self.lock:
        #     if len(self.pos_x) >= 2:
        #         all_runs.append({
        #             'x': list(self.pos_x),
        #             'y': list(self.pos_y),
        #             'run_id': self.run_id,
        #             'filename': f'{self.stage_name}_run{self.run_id:03d}.csv'
        #         })
        
        if not all_runs:
            print(f"[Recorder] No data to plot")
            return False
        
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Define colors for different runs
        colors = plt.cm.tab10.colors
        
        # Plot all runs
        for run in all_runs:
            color = colors[(run['run_id'] - 1) % len(colors)]
            ax.plot(run['x'], run['y'], '-', linewidth=1.5, color=color,
                   label=f'Run {run["run_id"]}', alpha=0.7)
        
        ax.set_title(f'2D Trajectory: {self.stage_name}\n{self.topic_name} ({len(all_runs)} runs)', fontsize=14)
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal', adjustable='datalim')
        #ax.legend(loc='upper left')
        
        plt.tight_layout()
        
        try:
            plt.savefig(self.pdf_path, format='pdf', dpi=150, bbox_inches='tight')
            print(f"[Recorder] PDF SAVED: {self.pdf_path}")
            print(f"[Recorder] Total runs in PDF: {len(all_runs)}")
        except Exception as e:
            print(f"[Recorder] ERROR saving PDF: {e}")
            return False
        finally:
            plt.close(fig)
        
        return True
    
    def stop(self):
        """Stop the recorder and save files."""
        print(f"[Recorder] Stopping...")
        if self.subscriber is not None:
            self.subscriber.unregister()
        self.save_csv()  # Save current run to CSV
        self.save_pdf()  # Generate PDF with all runs

def access_sensor_graph(cam_name, cam_state, lidar):
    stage = omni.usd.get_context().get_stage()
    sensor_graph_path = "/map/odom/odom/barakuda/SensorGraph"
    graph = og.get_graph_by_path(sensor_graph_path)

    # Mapping: topic prefix -> cam_name
    topic_to_cam = {
        "cam0/": "CameraLeft0",
        "cam1/": "CameraRight1",
        "cam2/": "CameraLeft2",
    }

    for node in graph.get_nodes():
        node_name = node.get_type_name()

        # Handle render product nodes
        if node_name == "isaacsim.core.nodes.IsaacCreateRenderProduct":
            cam_attr = node.get_attribute("inputs:cameraPrim")
            if cam_attr:
                camera_path = str(cam_attr.get())
                for i, name in enumerate(cam_name):
                    if name in camera_path:
                        enabled_attr = node.get_attribute("inputs:enabled")
                        if cam_state[i] == 0:
                            print(f"Disabling RenderProduct for {name}")
                            node.set_disabled(True)
                            if enabled_attr:
                                enabled_attr.set(False)
                        else:
                            print(f"Enabling RenderProduct for {name}")
                            node.set_disabled(False)
                            if enabled_attr:
                                enabled_attr.set(True)
                        break

        # Handle camera helper nodes
        if node_name == "isaacsim.ros1.bridge.ROS1CameraHelper":
            topic_attr = node.get_attribute("inputs:topicName")
            if topic_attr:
                topic = str(topic_attr.get())
                for prefix, cam in topic_to_cam.items():
                    if topic.startswith(prefix):
                        cam_idx = cam_name.index(cam) if cam in cam_name else -1
                        enabled_attr = node.get_attribute("inputs:enabled")
                        # Always disable depth_pcl topics
                        if "depth_pcl" in topic:
                            print(f"Disabling ROS1CameraHelper (depth_pcl): {topic}")
                            node.set_disabled(True)
                            if enabled_attr:
                                enabled_attr.set(False)
                        elif cam_idx >= 0 and cam_state[cam_idx] == 0:
                            print(f"Disabling ROS1CameraHelper: {topic}")
                            node.set_disabled(True)
                            if enabled_attr:
                                enabled_attr.set(False)
                        else:
                            print(f"Enabling ROS1CameraHelper: {topic}")
                            node.set_disabled(False)
                            if enabled_attr:
                                enabled_attr.set(True)
                        break

        # Handle lidar
        if node_name == "isaacsim.ros1.bridge.ROS1RtxLidarHelper" and lidar == 0:
            node.set_disabled(True)
            enabled_attr = node.get_attribute("inputs:enabled")
            if enabled_attr:
                enabled_attr.set(False)
            print("Disabling Lidar")

def run(path_stage, odom_topic="/imu/odometry", output_dir='/lvhome/run-stack-nav', sensor='lidar', record=False):
    global shutdown_requested

    # Ensure absolute path
    if not path_stage.startswith('/'):
        path_stage = '/' + path_stage

    # Load your USD
    open_stage(path_stage)
    
    # Create a World instance to manage the simulation
    world = World()

    # robot_prim_path = "/map/barakuda/barakuda/barakuda/barakuda"
    # robot_prim = get_prim_at_path(robot_prim_path)
    
    # Get the timeline interface and start simulation
    timeline = omni.timeline.get_timeline_interface()

    # 1 = sensor activated, 0 = sensor disabled
    if sensor == 'lidar' or sensor == 'mono':
        cam_name = ["CameraLeft0", "CameraLeft2", "CameraRight1"]
        cam_state = [0, 1, 0] # Left = "CameraLeft0", Middle = "CameraLeft2", Right = "CameraRight1"
        lidar = 1
    else:
        cam_name = ["CameraLeft0", "CameraLeft2", "CameraRight1"]
        cam_state = [1, 0, 1] # Left = "CameraLeft0", Middle = "CameraLeft2", Right = "CameraRight1"
        lidar = 1
        print(f"[DEBUG] Stereo mode: cam_state={cam_state}")
    # access_sensor_graph(cam_name, cam_state, lidar)  # Check sensor graph before starting simulation

    #rospy.init_node('isaac_sim_lidar', anonymous=True)

    # Generate unique output path
    recorder = None
    if record:
        base_stage_name = os.path.splitext(os.path.basename(path_stage))[0]
        stage_name = f"{sensor}_{base_stage_name}"

        # Initialize trajectory recorder
        recorder = TrajectoryRecorder(
            topic_name=odom_topic, 
            output_dir=output_dir,
            stage_name=stage_name
        )
        recorder.start_subscriber()
        print(f"Recording trajectory from: {odom_topic}")
        print(f"CSV will be saved to: {recorder.csv_path}")
        print(f"PDF will be saved to: {recorder.pdf_path}")
        print(f"Press Ctrl+C to stop and save the trajectory.")
    else:
        print("Trajectory recording disabled")

    timeline.play()
    # Let the simulation initialize
    for _ in range(10):
        world.step(render=True)

    access_sensor_graph(cam_name, cam_state, lidar)
    
    # Main loop - check for shutdown flag
    while simulation_app.is_running() and not shutdown_requested:
        world.step(render=True)
        time.sleep(0.033)
    
    # Cleanup
    if recorder is not None:
        print("[Main] Exiting main loop, saving data...")
        recorder.stop()
    simulation_app.close()
    print("[Main] Done.")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Isaac Sim with trajectory recording to PDF')
    parser.add_argument('--stage', '-s', required=True, help='Environment stage to load')
    parser.add_argument('--odom-topic', '-o', default='/imu/odometry', help='Odometry topic (default: /imu/odometry)')
    parser.add_argument('--output-dir', '-d', default='/lvhome/run-stack-nav', help='Output directory for PDF (default: current directory)')
    parser.add_argument('--sensor', type=str, choices=['lidar', 'mono', 'stereo'], help='Sensor type: lidar, mono, or stereo')
    parser.add_argument('--record', action='store_true', help='Enable trajectory recording')
    parser.add_argument('--gui', action='store_true', help='Run with Isaac Sim GUI (default: headless)')
    args = parser.parse_args()
    
    run(args.stage, odom_topic=args.odom_topic, output_dir=args.output_dir, sensor=args.sensor, record=args.record)