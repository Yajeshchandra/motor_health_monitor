import csv
import time
from abc import ABC, abstractmethod
import matplotlib.pyplot as plt
from openpyxl import Workbook
from pymavlink import mavutil
import numpy as np
from typing import List, Dict, Any, Tuple, Optional
from dataclasses import dataclass

@dataclass
class IMUData:
    """Data structure for IMU measurements"""
    timestamp: float
    gyro: Tuple[float, float, float]
    accel: Tuple[float, float, float]
    mag: Tuple[float, float, float]
    attitude: Tuple[float, float, float]
    mag_attitude: Tuple[float, float, float]
    mag_attitude_raw: Tuple[float, float, float]

class DataFilter(ABC):
    """Abstract base class for data filters"""
    @abstractmethod
    def process(self, data: List[float]) -> List[float]:
        pass

class MovingAverageFilter(DataFilter):
    """Simple moving average filter"""
    def __init__(self, window_size: int):
        self.window_size = window_size
        self.buffer = []

    def process(self, data: List[float]) -> List[float]:
        self.buffer.extend(data)
        filtered_data = []
        for i in range(len(self.buffer) - self.window_size + 1):
            window = self.buffer[i:i + self.window_size]
            filtered_data.append(sum(window) / self.window_size)
        self.buffer = self.buffer[-self.window_size:]
        return filtered_data

from filterpy.kalman import KalmanFilter
import numpy as np

class KalmanDataFilter:
    """Kalman filter using filterpy library."""
    
    def __init__(self, process_variance: float, measurement_variance: float):
        # Initialize the KalmanFilter object
        self.kf = KalmanFilter(dim_x=1, dim_z=1)  # 1D Kalman filter
        self.kf.x = np.array([[0.]])  # Initial estimate
        self.kf.F = np.array([[1.]])  # State transition matrix
        self.kf.H = np.array([[1.]])  # Measurement function
        self.kf.P = np.array([[1.]])  # Estimate uncertainty (initial estimate error)
        self.kf.R = np.array([[measurement_variance]])  # Measurement uncertainty
        self.kf.Q = np.array([[process_variance]])  # Process uncertainty

    def process(self, data: list) -> list:
        filtered_data = []
        for measurement in data:
            # Predict the next state (not really needed in this simple 1D case, but part of Kalman filter)
            self.kf.predict()

            # Update with the measurement
            self.kf.update(np.array([[measurement]]))

            # Store the filtered estimate
            filtered_data.append(self.kf.x[0][0])
        
        return filtered_data

class DataVisualizer:
    """Handles all visualization tasks"""
    def __init__(self, num_rows: int = 3, num_cols: int = 3):
        plt.ion()
        self.fig, self.axes = plt.subplots(num_rows, num_cols, figsize=(15, 12))
        self.plots = {}

    def add_plot(self, name: str, position: Tuple[int, int], title: str, xlabel: str, ylabel: str):
        """Add a new plot to the visualization"""
        self.plots[name] = {
            'position': position,
            'title': title,
            'xlabel': xlabel,
            'ylabel': ylabel,
            'lines': {}
        }

    def update_plot(self, plot_name: str, data_dict: Dict[str, Tuple[List[float], str]]):
        """Update a specific plot with new data"""
        plot = self.plots[plot_name]
        ax = self.axes[plot['position']]
        ax.clear()
        
        for label, (data, color) in data_dict.items():
            ax.plot(data, label=label, color=color)
        
        ax.set_title(plot['title'])
        ax.set_xlabel(plot['xlabel'])
        ax.set_ylabel(plot['ylabel'])
        ax.legend()

    def refresh(self):
        """Refresh the visualization"""
        plt.tight_layout()
        plt.pause(0.01)

class DataLogger:
    """Handles data logging to CSV and Excel"""
    def __init__(self, csv_filename: str, excel_filename: str):
        self.csv_filename = csv_filename
        self.excel_filename = excel_filename
        self.wb = Workbook()
        self.ws = self.wb.active
        self.setup_files()

    def setup_files(self):
        """Set up the CSV and Excel files with headers"""
        headers = [
            "Timestamp",
            "Gyro_X", "Gyro_Y", "Gyro_Z",
            "Accel_X", "Accel_Y", "Accel_Z",
            "Mag_X", "Mag_Y", "Mag_Z",
            "Roll", "Pitch", "Yaw",
            "Roll_Mag", "Pitch_Mag", "Yaw_Mag",
            "Roll_Mag_Raw", "Pitch_Mag_Raw", "Yaw_Mag_Raw"
        ]
        
        with open(self.csv_filename, 'w', newline='') as f:
            csv.writer(f).writerow(headers)
        
        self.ws.append(headers)
        self.wb.save(self.excel_filename)

    def log_data(self, data: IMUData):
        """Log the IMU data to both CSV and Excel"""
        row = [
            data.timestamp,
            *data.gyro, *data.accel, *data.mag,
            *data.attitude, *data.mag_attitude, *data.mag_attitude_raw
        ]
        
        with open(self.csv_filename, 'a', newline='') as f:
            csv.writer(f).writerow(row)
        
        self.ws.append(row)
        self.wb.save(self.excel_filename)

class IMUDataCollector:
    """Main class for collecting and processing IMU data"""
    def __init__(self, connection_string: str):
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print(f"Heartbeat from system (system_id: {self.master.target_system})")
        
        self.data_buffer = []
        self.filters = {}
        self.visualizer = DataVisualizer()
        self.logger = DataLogger('imu_data.csv', 'imu_data.xlsx')
        self.setup_visualization()

    def add_filter(self, name: str, filter_instance: DataFilter):
        """Add a new filter to process data"""
        self.filters[name] = filter_instance

    def setup_visualization(self):
        """Set up the default visualization plots"""
        self.visualizer.add_plot('gyro', (0, 0), 'Gyroscope Data', 'Time', 'rad/s')
        self.visualizer.add_plot('accel', (0, 1), 'Accelerometer Data', 'Time', 'm/s²')
        self.visualizer.add_plot('mag', (0, 2), 'Magnetometer Data', 'Time', 'Gauss')
        self.visualizer.add_plot('attitude', (1, 0), 'Attitude Data', 'Time', 'rad')
        self.visualizer.add_plot('mag_attitude', (1, 1), 'Mag Attitude', 'Time', 'rad')
        self.visualizer.add_plot('mag_raw', (1, 2), 'Mag Raw Data', 'Time', 'rad')

    def calculate_attitude_from_mag(self, mag: Tuple[float, float, float], 
                                 accel: Tuple[float, float, float]) -> Tuple[float, float, float, float, float, float]:
        """Calculate attitude from magnetometer data"""
        mag_x, mag_y, mag_z = mag
        accel_x, accel_y, accel_z = accel
        
        # Your existing attitude calculation code here
        # Returning placeholder values for brevity
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def collect_data(self):
        """Main data collection loop"""
        try:
            while True:
                msg_imu = self.master.recv_match(type='HIGHRES_IMU', blocking=True)
                msg_att = self.master.recv_match(type='ATTITUDE', blocking=True)

                if msg_imu and msg_att:
                    timestamp = time.time()
                    
                    # Collect raw data
                    gyro = (msg_imu.xgyro, msg_imu.ygyro, msg_imu.zgyro)
                    accel = (msg_imu.xacc, msg_imu.yacc, msg_imu.zacc)
                    mag = (msg_imu.xmag, msg_imu.ymag, msg_imu.zmag)
                    attitude = (msg_att.roll, msg_att.pitch, msg_att.yaw)
                    
                    # Calculate magnetometer-based attitude
                    mag_att_data = self.calculate_attitude_from_mag(mag, accel)
                    mag_attitude = mag_att_data[:3]
                    mag_attitude_raw = mag_att_data[3:]

                    # Create IMU data object
                    imu_data = IMUData(
                        timestamp=timestamp,
                        gyro=gyro,
                        accel=accel,
                        mag=mag,
                        attitude=attitude,
                        mag_attitude=mag_attitude,
                        mag_attitude_raw=mag_attitude_raw
                    )

                    # Store data
                    self.data_buffer.append(imu_data)
                    self.logger.log_data(imu_data)

                    # Apply filters and update visualization
                    self.update_visualization()

        except KeyboardInterrupt:
            print("Stopping data collection...")

    def update_visualization(self):
        """Update all visualization plots"""
        # Example of updating gyro plot with filtered and raw data
        timestamps = [data.timestamp for data in self.data_buffer]
        gyro_x = [data.gyro[0] for data in self.data_buffer]
        
        # Apply filters if they exist
        filtered_gyro_x = gyro_x
        if 'moving_average' in self.filters:
            filtered_gyro_x = self.filters['moving_average'].process(gyro_x)

        self.visualizer.update_plot('gyro', {
            'Raw': (gyro_x, 'blue'),
            'Filtered': (filtered_gyro_x, 'red')
        })
        
        # Update other plots similarly
        self.visualizer.refresh()

def main():
    # Example usage
    collector = IMUDataCollector('udp:127.0.0.1:14540')
    
    # Add filters
    collector.add_filter('moving_average', MovingAverageFilter(window_size=5))
    collector.add_filter('kalman', KalmanFilter(process_variance=1e-5, measurement_variance=1e-3))
    
    # Start collection
    collector.collect_data()

if __name__ == "__main__":
    main()