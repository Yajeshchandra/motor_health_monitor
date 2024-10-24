import csv
import time
import matplotlib.pyplot as plt
from openpyxl import Workbook
from pymavlink import mavutil
import numpy as np

# Create a connection to the PX4 (change the connection string as needed)
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')  # For SITL

# Wait for the first heartbeat
master.wait_heartbeat()
print("Heartbeat from system (system_id: {})".format(master.target_system))

# Prepare the plot
plt.ion()  # Enable interactive mode
fig, ((ax1, ax2, ax3),(ax4, ax5, ax6)) = plt.subplots(2, 3, figsize=(10, 12))  # Added fourth subplot for mag data
gyro_x_data, gyro_y_data, gyro_z_data = [], [], []
accel_x_data, accel_y_data, accel_z_data = [], [], []
mag_x_data, mag_y_data, mag_z_data = [], [], []  # Added magnetometer data lists
roll_data, pitch_data, yaw_data = [], [], []
roll_data_mag = []  # Added magnetometer roll data list
roll_data_mag_raw = []
pitch_data_mag = []  # Added magnetometer pitch data list
pitch_data_mag_raw = []
timestamps = []

import numpy as np

def calculate_thrust_vector_angle(roll, pitch):
    """
    Calculate the angle between the thrust vector and the Z axis using roll and pitch angles.
    
    Args:
        roll (float): Roll angle in radians
        pitch (float): Pitch angle in radians
    
    Returns:
        float: Angle between thrust vector and Z axis in radians
    """
    # Create rotation matrices for roll and pitch
    R_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    R_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Initial thrust vector aligned with Z axis
    thrust_vector = np.array([0, 0, 1])
    
    # Apply rotations to get the actual thrust vector
    rotated_vector = R_pitch @ R_roll @ thrust_vector
    
    # Calculate angle between rotated vector and Z axis
    z_axis = np.array([0, 0, 1])
    angle = np.arccos(np.dot(rotated_vector, z_axis))
    
    return angle

# Function to get thrust vector components
def get_thrust_vector_components(roll, pitch):
    """
    Get the X, Y, Z components of the thrust vector.
    
    Args:
        roll (float): Roll angle in radians
        pitch (float): Pitch angle in radians
    
    Returns:
        tuple: (x, y, z) components of the thrust vector
    """
    R_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    R_pitch = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    thrust_vector = np.array([0, 0, 1])
    rotated_vector = R_pitch @ R_roll @ thrust_vector
    
    return tuple(rotated_vector)


def calculate_attitude_from_mag(mag_x, mag_y, mag_z, accel_x, accel_y, accel_z):
    """
    Calculate roll and pitch angles using magnetometer data with tilt compensation from accelerometer.
    
    Args:
        mag_x, mag_y, mag_z: Magnetometer readings in Gauss
        accel_x, accel_y, accel_z: Accelerometer readings in m/s^2
    
    Returns:
        roll_angle, pitch_angle: Roll and pitch angles in radians
    """
    # First calculate pitch and roll from accelerometer (initial estimates)
    roll_acc = np.arctan2(accel_y, np.sqrt(accel_x**2 + accel_z**2))
    pitch_acc = np.arctan2(-accel_x, np.sqrt(accel_y**2 + accel_z**2))
    
    # Tilt compensation calculations
    cos_roll = np.cos(roll_acc)
    sin_roll = np.sin(roll_acc)
    cos_pitch = np.cos(pitch_acc)
    sin_pitch = np.sin(pitch_acc)
    
    # Tilt compensated magnetic field components
    mag_x_comp = (mag_x * cos_pitch + 
                  mag_y * sin_roll * sin_pitch + 
                  mag_z * cos_roll * sin_pitch)
    
    mag_y_comp = (mag_y * cos_roll - 
                  mag_z * sin_roll)
    
    mag_z_comp = (-mag_x * sin_pitch +
                   mag_y * sin_roll * cos_pitch +
                   mag_z * cos_roll * cos_pitch)
    
    # Calculate roll and pitch from compensated magnetometer readings
    roll_mag = np.arctan2(-mag_y_comp, mag_x_comp)
    pitch_mag = np.arctan2(mag_z_comp, np.sqrt(mag_x_comp**2 + mag_y_comp**2))
    
    # Ensure angles are in the range [-pi, pi]
    roll_mag = np.mod(roll_mag + np.pi, 2 * np.pi) - np.pi
    pitch_mag = np.mod(pitch_mag + np.pi, 2 * np.pi) - np.pi
    
    # Complementary filter to combine accelerometer and magnetometer data
    alpha_roll = 0.95  # Filter coefficient for roll
    alpha_pitch = 0.95  # Filter coefficient for pitch
    
    roll_angle = alpha_roll * roll_acc + (1 - alpha_roll) * roll_mag
    pitch_angle = alpha_pitch * pitch_acc + (1 - alpha_pitch) * pitch_mag
    
    return roll_angle, pitch_angle, roll_mag, pitch_mag

# Open the CSV file for writing
with open('imu_data.csv', mode='w', newline='') as csv_file:
    # Create a CSV writer object
    csv_writer = csv.writer(csv_file)
    
    # Write the header row to the CSV file (added mag data columns)
    csv_writer.writerow([
        "Timestamp", "Gyro_X", "Gyro_Y", "Gyro_Z",
        "Accel_X", "Accel_Y", "Accel_Z",
        "Mag_X", "Mag_Y", "Mag_Z",  # Added magnetometer columns
        "Roll", "Pitch", "Yaw",
        "Roll_Mag", "Pitch_Mag",
        "Roll_Mag_Raw", "Pitch_Mag_Raw"
    ])
    
    # Create an Excel workbook and worksheet
    wb = Workbook()
    ws = wb.active
    ws.title = "IMU and Attitude Data"
    
    # Write the header row to the Excel file
    ws.append([
        "Timestamp", "Gyro_X", "Gyro_Y", "Gyro_Z",
        "Accel_X", "Accel_Y", "Accel_Z",
        "Mag_X", "Mag_Y", "Mag_Z",  # Added magnetometer columns
        "Roll", "Pitch", "Yaw",
        "Roll_Mag", "Pitch_Mag",
        "Roll_Mag_Raw", "Pitch_Mag_Raw"
    ])

    try:
        while True:
            # Listen for HIGHRES_IMU messages to get gyroscope and accelerometer data
            msg_imu = master.recv_match(type='HIGHRES_IMU', blocking=True)
            msg_att = master.recv_match(type='ATTITUDE', blocking=True)
            
            if msg_imu and msg_att:
                # Get the current timestamp
                timestamp = time.time()

                # Gyroscope readings (rad/s)
                gyro_x = msg_imu.xgyro
                gyro_y = msg_imu.ygyro
                gyro_z = msg_imu.zgyro
                print(f"Gyro: X={gyro_x:.2f} Y={gyro_y:.2f} Z={gyro_z:.2f}")

                # Accelerometer readings (m/s^2)
                accel_x = msg_imu.xacc
                accel_y = msg_imu.yacc
                accel_z = msg_imu.zacc
                print(f"Accel: X={accel_x:.2f} Y={accel_y:.2f} Z={accel_z:.2f}")

                # Magnetometer readings (Gauss)
                mag_x = msg_imu.xmag
                mag_y = msg_imu.ymag
                mag_z = msg_imu.zmag
                print(f"Mag: X={mag_x:.2f} Y={mag_y:.2f} Z={mag_z:.2f}")  # Added magnetometer print

                # Attitude readings (roll, pitch, yaw in radians)
                roll = msg_att.roll
                pitch = msg_att.pitch
                yaw = msg_att.yaw
                print(f"Attitude: Roll={roll:.2f} Pitch={pitch:.2f} Yaw={yaw:.2f}")
                
                roll_mag, pitch_mag, roll_mag_raw, pitch_mag_raw = calculate_attitude_from_mag(mag_x, mag_y, mag_z, accel_x, accel_y, accel_z)

                # Append data to lists for plotting
                timestamps.append(timestamp)
                gyro_x_data.append(gyro_x)
                gyro_y_data.append(gyro_y)
                gyro_z_data.append(gyro_z)
                accel_x_data.append(accel_x)
                accel_y_data.append(accel_y)
                accel_z_data.append(accel_z)
                mag_x_data.append(mag_x)  # Added magnetometer data
                mag_y_data.append(mag_y)
                mag_z_data.append(mag_z)
                roll_data.append(roll)
                pitch_data.append(pitch)
                yaw_data.append(yaw)
                roll_data_mag.append(roll_mag)
                pitch_data_mag.append(pitch_mag)
                roll_data_mag_raw.append(roll_mag_raw)
                pitch_data_mag_raw.append(pitch_mag_raw)

                # Write the data to the CSV file (added mag data)
                csv_writer.writerow([
                    timestamp,
                    gyro_x, gyro_y, gyro_z,
                    accel_x, accel_y, accel_z,
                    mag_x, mag_y, mag_z,  # Added magnetometer data
                    roll, pitch, yaw,
                    roll_mag, pitch_mag,
                    roll_mag_raw, pitch_mag_raw
                ])

                # Write the same data to the Excel file
                ws.append([
                    timestamp,
                    gyro_x, gyro_y, gyro_z,
                    accel_x, accel_y, accel_z,
                    mag_x, mag_y, mag_z,  # Added magnetometer data
                    roll, pitch, yaw,
                    roll_mag, pitch_mag,
                    roll_mag_raw, pitch_mag_raw
                ])

                # Save the Excel file after every new entry to avoid data loss
                wb.save("imu_data.xlsx")

                # Plot the Gyro data
                ax1.clear()
                ax1.plot(timestamps, gyro_x_data, label='Gyro X', color='r')
                ax1.plot(timestamps, gyro_y_data, label='Gyro Y', color='g')
                ax1.plot(timestamps, gyro_z_data, label='Gyro Z', color='b')
                ax1.set_title('Gyroscope Data (rad/s)')
                ax1.set_xlabel('Time (s)')
                ax1.set_ylabel('Gyro (rad/s)')
                ax1.legend()

                # Plot the Accel data
                ax2.clear()
                ax2.plot(timestamps, accel_x_data, label='Accel X', color='r')
                ax2.plot(timestamps, accel_y_data, label='Accel Y', color='g')
                ax2.plot(timestamps, accel_z_data, label='Accel Z', color='b')
                ax2.set_title('Accelerometer Data (m/s^2)')
                ax2.set_xlabel('Time (s)')
                ax2.set_ylabel('Accel (m/s^2)')
                ax2.legend()

                # Plot the Magnetometer data
                ax3.clear()
                ax3.plot(timestamps, mag_x_data, label='Mag X', color='r')
                ax3.plot(timestamps, mag_y_data, label='Mag Y', color='g')
                ax3.plot(timestamps, mag_z_data, label='Mag Z', color='b')
                ax3.set_title('Magnetometer Data (Gauss)')
                ax3.set_xlabel('Time (s)')
                ax3.set_ylabel('Magnetic Field (Gauss)')
                ax3.legend()

                # Plot the Attitude data (Roll, Pitch, Yaw)
                ax4.clear()
                ax4.plot(timestamps, roll_data, label='Roll Cal', color='r')
                ax4.plot(timestamps, roll_data_mag, label='Roll Cal', color='r')
                ax4.set_title('Roll Data (rad)')
                ax4.set_xlabel('Time (s)')
                ax4.set_ylabel('Angle (rad)')
                ax4.legend()

                ax5.clear()
                ax5.plot(timestamps, pitch_data, label='Pitch Cal', color='g')
                ax5.plot(timestamps, pitch_data_mag, label='Pitch Cal', color='g')
                ax5.set_title('Pitch Data (rad)')
                ax5.set_xlabel('Time (s)')
                ax5.set_ylabel('Angle (rad)')
                ax5.legend()
                
                ax6.clear()
                ax6.plot(timestamps, yaw_data, label='Yaw Cal', color='b')
                ax6.set_title('Yaw Data (rad)')
                ax6.set_xlabel('Time (s)')
                ax6.set_ylabel('Angle (rad)')
                ax6.legend()

                # Adjust layout to prevent subplot overlap
                plt.tight_layout()

                # Update the plot
                plt.pause(0.01)
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Saving data and exiting...")
        wb.save("imu_data.xlsx")
        csv_file.close()