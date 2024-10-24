import csv
import time
import numpy as np
import matplotlib.pyplot as plt
from openpyxl import Workbook
from pymavlink import mavutil
from collections import deque
from threading import Thread, Lock
from queue import Queue

class IMUMonitor:
    def __init__(self, connection_string='udp:127.0.0.1:14540'):
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print(f"Heartbeat from system (system_id: {self.master.target_system})")
        
        # Data queues for async processing
        self.imu_queue = Queue(maxsize=1000)
        self.attitude_queue = Queue(maxsize=1000)
        
        # Data storage with fixed length for real-time analysis
        self.window_size = 100  # Adjust based on your needs
        self.accel_history = deque(maxlen=self.window_size)
        self.gyro_history = deque(maxlen=self.window_size)
        self.mag_history = deque(maxlen=self.window_size)
        
        # Failure detection parameters
        self.accel_threshold = 15.0  # m/s^2
        self.gyro_threshold = 5.0    # rad/s
        self.sudden_change_threshold = 0.8  # 80% change
        
        # Thread safety
        self.data_lock = Lock()
        self.running = True
        
        # Initialize plots
        self.setup_plots()
        
        # Start background threads
        self.start_threads()

    def setup_plots(self):
        plt.ion()
        self.fig, self.axes = plt.subplots(4, 1, figsize=(12, 10))
        self.fig.canvas.manager.set_window_title('Real-time IMU Monitor')
        
    def start_threads(self):
        # Start data collection threads
        self.imu_thread = Thread(target=self.collect_imu_data, daemon=True)
        self.attitude_thread = Thread(target=self.collect_attitude_data, daemon=True)
        self.processing_thread = Thread(target=self.process_data, daemon=True)
        
        self.imu_thread.start()
        self.attitude_thread.start()
        self.processing_thread.start()

    def collect_imu_data(self):
        """Collect IMU data at maximum rate"""
        while self.running:
            msg = self.master.recv_match(type='HIGHRES_IMU', blocking=True)
            if msg:
                self.imu_queue.put((time.time(), msg))

    def collect_attitude_data(self):
        """Collect attitude data at maximum rate"""
        while self.running:
            msg = self.master.recv_match(type='ATTITUDE', blocking=True)
            if msg:
                self.attitude_queue.put((time.time(), msg))

    def detect_anomalies(self, data_history, new_data, threshold):
        """Detect sudden changes in sensor data"""
        if len(data_history) < 2:
            return False
            
        # Calculate rate of change
        previous = np.array(data_history[-1])
        current = np.array(new_data)
        delta = np.abs(current - previous)
        
        # Check for sudden changes
        if np.any(delta > threshold):
            return True
            
        # Check for unusual patterns
        if len(data_history) >= 10:
            recent_mean = np.mean(list(data_history)[-10:], axis=0)
            current_deviation = np.abs(current - recent_mean)
            if np.any(current_deviation > threshold * 2):
                return True
                
        return False

    def predict_failure(self):
        """Predict potential failures before they occur"""
        with self.data_lock:
            if len(self.accel_history) < 10:
                return False, "Insufficient data"
                
            # Convert deque to numpy array for vectorized operations
            accel_data = np.array(list(self.accel_history))
            gyro_data = np.array(list(self.gyro_history))
            
            # Check acceleration trends
            accel_magnitude = np.linalg.norm(accel_data, axis=1)
            accel_trend = np.gradient(accel_magnitude)
            
            # Check rotation rates
            gyro_magnitude = np.linalg.norm(gyro_data, axis=1)
            gyro_trend = np.gradient(gyro_magnitude)
            
            # Failure prediction logic
            potential_failure = False
            failure_reason = []
            
            # Check for free fall (near-zero acceleration)
            if np.mean(accel_magnitude[-5:]) < 2.0:  # Less than 2 m/s²
                potential_failure = True
                failure_reason.append("Possible free fall detected")
                
            # Check for excessive rotation
            if np.mean(gyro_magnitude[-5:]) > self.gyro_threshold:
                potential_failure = True
                failure_reason.append("Excessive rotation detected")
                
            # Check for sudden acceleration changes
            if np.any(np.abs(accel_trend[-5:]) > self.accel_threshold):
                potential_failure = True
                failure_reason.append("Sudden acceleration change")
                
            return potential_failure, " | ".join(failure_reason)

    def process_data(self):
        """Process incoming data and update visualizations"""
        csv_file = open('imu_data_high_rate.csv', 'w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            "Timestamp", "Accel_X", "Accel_Y", "Accel_Z",
            "Gyro_X", "Gyro_Y", "Gyro_Z",
            "Mag_X", "Mag_Y", "Mag_Z",
            "Roll", "Pitch", "Yaw",
            "Failure_Predicted", "Failure_Reason"
        ])

        last_plot_time = time.time()
        plot_interval = 0.05  # 20 Hz plot update

        while self.running:
            try:
                # Process IMU data
                if not self.imu_queue.empty():
                    timestamp, imu_msg = self.imu_queue.get_nowait()
                    
                    # Extract and store sensor data
                    accel_data = [imu_msg.xacc, imu_msg.yacc, imu_msg.zacc]
                    gyro_data = [imu_msg.xgyro, imu_msg.ygyro, imu_msg.zgyro]
                    mag_data = [imu_msg.xmag, imu_msg.ymag, imu_msg.zmag]
                    
                    with self.data_lock:
                        self.accel_history.append(accel_data)
                        self.gyro_history.append(gyro_data)
                        self.mag_history.append(mag_data)
                    
                    # Predict failures
                    failure_predicted, failure_reason = self.predict_failure()
                    
                    # High-rate console output for critical changes
                    if failure_predicted:
                        print(f"\033[91mWARNING: {failure_reason}\033[0m")  # Red text
                        print(f"Acceleration: {accel_data}")
                        print(f"Angular rates: {gyro_data}")
                
                # Update plots at controlled rate
                current_time = time.time()
                if current_time - last_plot_time >= plot_interval:
                    self.update_plots()
                    last_plot_time = current_time
                    
            except Exception as e:
                print(f"Error in data processing: {e}")
                continue

    def update_plots(self):
        """Update visualization plots"""
        with self.data_lock:
            if len(self.accel_history) < 2:
                return
                
            # Convert deques to numpy arrays for plotting
            accel_data = np.array(list(self.accel_history))
            gyro_data = np.array(list(self.gyro_history))
            mag_data = np.array(list(self.mag_history))
            
            # Clear all axes
            for ax in self.axes:
                ax.clear()
            
            # Plot accelerometer data
            self.axes[0].plot(accel_data[:, 0], 'r-', label='X')
            self.axes[0].plot(accel_data[:, 1], 'g-', label='Y')
            self.axes[0].plot(accel_data[:, 2], 'b-', label='Z')
            self.axes[0].set_title('Accelerometer Data')
            self.axes[0].legend()
            
            # Plot gyroscope data
            self.axes[1].plot(gyro_data[:, 0], 'r-', label='X')
            self.axes[1].plot(gyro_data[:, 1], 'g-', label='Y')
            self.axes[1].plot(gyro_data[:, 2], 'b-', label='Z')
            self.axes[1].set_title('Gyroscope Data')
            self.axes[1].legend()
            
            # Plot magnetometer data
            self.axes[2].plot(mag_data[:, 0], 'r-', label='X')
            self.axes[2].plot(mag_data[:, 1], 'g-', label='Y')
            self.axes[2].plot(mag_data[:, 2], 'b-', label='Z')
            self.axes[2].set_title('Magnetometer Data')
            self.axes[2].legend()
            
            # Plot combined magnitude
            accel_mag = np.linalg.norm(accel_data, axis=1)
            gyro_mag = np.linalg.norm(gyro_data, axis=1)
            self.axes[3].plot(accel_mag, 'r-', label='Accel Magnitude')
            self.axes[3].plot(gyro_mag, 'b-', label='Gyro Magnitude')
            self.axes[3].set_title('Sensor Magnitudes')
            self.axes[3].legend()
            
            plt.tight_layout()
            plt.pause(0.01)

    def run(self):
        """Main run loop"""
        try:
            while self.running:
                time.sleep(0.1)  # Small sleep to prevent CPU overload
        except KeyboardInterrupt:
            self.running = False
            print("\nShutting down IMU monitor...")
        finally:
            plt.close('all')

# Usage example
if __name__ == "__main__":
    monitor = IMUMonitor()
    monitor.run()