import time
from pymavlink import mavutil
import numpy as np
from datetime import datetime
import csv

csvfile=None

class HighResPX4Monitor:
    def __init__(self, connection_string='udp:127.0.0.1:14540'):
        # Initialize connection
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()
        print(f"Connected to system (system_id: {self.master.target_system})")
        self.fail = False
        self.motor = 0
        
        # Request data streams at maximum rate
        messages_to_request = [
            mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW
        ]
        
        for msg_id in messages_to_request:
            self.request_message_interval(msg_id, 1000)
        
        # Initialize CSV writer
        # timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        global csvfile
        self.csv_file = open(f'px4_data.csv', 'w', newline='')
        csvfile = self.csv_file
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write header
        self.csv_writer.writerow([
            'Timestamp_ns',
            # IMU Data
            'Gyro_X', 'Gyro_Y', 'Gyro_Z',
            'Acc_X', 'Acc_Y', 'Acc_Z',
            'Mag_X', 'Mag_Y', 'Mag_Z',
            # Attitude
            'Roll', 'Pitch', 'Yaw',
            # Position
            'Pos_X', 'Pos_Y', 'Pos_Z',
            'Vel_X', 'Vel_Y', 'Vel_Z',
            # VFR HUD
            'Airspeed', 'Groundspeed', 'Heading',
            'Throttle', 'Alt', 'Climb',
            # Servo Outputs
            'Servo1_Raw', 'Servo2_Raw', 'Servo3_Raw', 'Servo4_Raw',
            # Calculated
            'Thrust_Vector_X', 'Thrust_Vector_Y', 'Thrust_Vector_Z',
            'Thrust_Angle_From_Z'
        ])

    def request_message_interval(self, message_id, frequency_hz):
        """Request a message at a particular frequency."""
        interval_us = 1000000 // frequency_hz
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, interval_us, 0, 0, 0, 0, 0)

    def calculate_thrust_vector(self, roll, pitch, yaw, throttle):
        """Calculate thrust vector components and angle from Z axis."""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R = np.array([
            [cp*cy, -cp*sy, sp],
            [sr*sp*cy + cr*sy, -sr*sp*sy + cr*cy, -sr*cp],
            [-cr*sp*cy + sr*sy, cr*sp*sy + sr*cy, cr*cp]
        ])
        
        # Scale thrust vector by throttle percentage
        thrust_magnitude = throttle / 100.0
        thrust_vector = np.array([0, 0, thrust_magnitude])
        
        rotated_vector = R @ thrust_vector
        
        z_axis = np.array([0, 0, 1])
        angle = np.arccos(np.clip(np.dot(rotated_vector, z_axis), -1.0, 1.0))
        
        return rotated_vector, angle

    def run(self):
        """Main loop to collect and process data."""
        try:
            while True:
                timestamp_ns = time.time_ns()
                
                # Initialize data containers
                imu_data = None
                attitude_data = None
                local_pos_data = None
                vfr_data = None
                servo_data = None
                
                timeout = time.time() + 0.1
                
                while time.time() < timeout:
                    msg = self.master.recv_match(blocking=True, timeout=0.01)
                    if msg is None:
                        continue
                        
                    msg_type = msg.get_type()
                    
                    if msg_type == 'HIGHRES_IMU':
                        imu_data = msg
                    elif msg_type == 'ATTITUDE':
                        attitude_data = msg
                    elif msg_type == 'LOCAL_POSITION_NED':
                        local_pos_data = msg
                    elif msg_type == 'VFR_HUD':
                        vfr_data = msg
                    elif msg_type == 'SERVO_OUTPUT_RAW':
                        servo_data = msg
                        
                    if all([imu_data, attitude_data, local_pos_data, vfr_data, servo_data]):
                        break
                
                if not all([imu_data, attitude_data, local_pos_data, vfr_data, servo_data]):
                    print("Timeout waiting for messages")
                    continue
                
                # Calculate thrust vector using throttle information
                thrust_vector, thrust_angle = self.calculate_thrust_vector(
                    attitude_data.roll,
                    attitude_data.pitch,
                    attitude_data.yaw,
                    vfr_data.throttle
                )
                
                if not self.fail:
                    roll = imu_data.xgyro
                    pitch = imu_data.ygyro
                
                if (roll>=2 or pitch>=2):
                    self.fail = True
                    if (roll>=0 and pitch>=0):
                        self.motor = 4
                    if (roll>=0 and pitch<=0):
                        self.motor = 3
                    if (roll<=0 and pitch<=0):
                        self.motor = 1
                    if (roll<=0 and pitch>=0):
                        self.motor = 2
                
                # Print formatted data
                print("\033[2J\033[H")  # Clear screen
                print(f"Timestamp: {timestamp_ns}")
                
                print("\nIMU Data:")
                print(f"Gyro (rad/s): X={imu_data.xgyro:8.4f} Y={imu_data.ygyro:8.4f} Z={imu_data.zgyro:8.4f}")
                print(f"Acc (m/s²):   X={imu_data.xacc:8.4f} Y={imu_data.yacc:8.4f} Z={imu_data.zacc:8.4f}")
                print(f"Mag (gauss):  X={imu_data.xmag:8.4f} Y={imu_data.ymag:8.4f} Z={imu_data.zmag:8.4f}")
                
                print("\nAttitude (rad):")
                print(f"Roll={attitude_data.roll:8.4f} Pitch={attitude_data.pitch:8.4f} Yaw={attitude_data.yaw:8.4f}")
                
                print("\nPosition (m) and Velocity (m/s):")
                print(f"Pos: X={local_pos_data.x:8.3f} Y={local_pos_data.y:8.3f} Z={local_pos_data.z:8.3f}")
                print(f"Vel: X={local_pos_data.vx:8.3f} Y={local_pos_data.vy:8.3f} Z={local_pos_data.vz:8.3f}")
                
                print("\nFlight Data:")
                print(f"Throttle={vfr_data.throttle:3.0f}% Airspeed={vfr_data.airspeed:6.2f}m/s Alt={vfr_data.alt:6.2f}m")
                
                print("\nMotor Outputs (raw):")
                print(f"M1={servo_data.servo1_raw:4d} M2={servo_data.servo2_raw:4d}")
                print(f"M3={servo_data.servo3_raw:4d} M4={servo_data.servo4_raw:4d}")
                
                print("\nThrust Vector:")
                print(f"Vector: X={thrust_vector[0]:8.4f} Y={thrust_vector[1]:8.4f} Z={thrust_vector[2]:8.4f}")
                print(f"Angle from Z axis: {np.degrees(thrust_angle):8.4f} degrees")
                
                if self.fail:
                    print("[WARN] : FAILURE")
                    print(f"[WARN] : Motor {self.motor} experienced failure")
                
                # Write to CSV
                self.csv_writer.writerow([
                    timestamp_ns,
                    # IMU
                    imu_data.xgyro, imu_data.ygyro, imu_data.zgyro,
                    imu_data.xacc, imu_data.yacc, imu_data.zacc,
                    imu_data.xmag, imu_data.ymag, imu_data.zmag,
                    # Attitude
                    attitude_data.roll, attitude_data.pitch, attitude_data.yaw,
                    # Position
                    local_pos_data.x, local_pos_data.y, local_pos_data.z,
                    local_pos_data.vx, local_pos_data.vy, local_pos_data.vz,
                    # VFR HUD
                    vfr_data.airspeed, vfr_data.groundspeed, vfr_data.heading,
                    vfr_data.throttle, vfr_data.alt, vfr_data.climb,
                    # Servo
                    servo_data.servo1_raw, servo_data.servo2_raw, 
                    servo_data.servo3_raw, servo_data.servo4_raw,
                    # Calculated
                    *thrust_vector,
                    thrust_angle
                ])
                
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\nExiting...")
            self.csv_file.close()
        except Exception as e:
            print(f"Error: {e}")
            self.csv_file.close()
            
import matplotlib.pyplot as plt

def plot_data(csv_filename):
    # Load the CSV data
    timestamps = []
    gyro_x, gyro_y, gyro_z = [], [], []
    acc_x, acc_y, acc_z = [], [], []
    roll, pitch, yaw = [], [], []
    thrust_vector_x, thrust_vector_y, thrust_vector_z = [], [], []
    thrust_angle_from_z = []
    
    with open(csv_filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            timestamps.append(float(row['Timestamp_ns']))
            gyro_x.append(float(row['Gyro_X']))
            gyro_y.append(float(row['Gyro_Y']))
            gyro_z.append(float(row['Gyro_Z']))
            acc_x.append(float(row['Acc_X']))
            acc_y.append(float(row['Acc_Y']))
            acc_z.append(float(row['Acc_Z']))
            roll.append(float(row['Roll']))
            pitch.append(float(row['Pitch']))
            yaw.append(float(row['Yaw']))
            thrust_vector_x.append(float(row['Thrust_Vector_X']))
            thrust_vector_y.append(float(row['Thrust_Vector_Y']))
            thrust_vector_z.append(float(row['Thrust_Vector_Z']))
            thrust_angle_from_z.append(float(row['Thrust_Angle_From_Z']))
    
    # Convert timestamps to seconds
    timestamps = np.array(timestamps) / 1e9  # Convert from ns to s
    
    # Plot gyro data
    plt.figure(figsize=(10, 6))
    plt.subplot(3, 1, 1)
    plt.plot(timestamps, gyro_x, label='Gyro X')
    plt.plot(timestamps, gyro_y, label='Gyro Y')
    plt.plot(timestamps, gyro_z, label='Gyro Z')
    plt.title('Gyroscope Data')
    plt.xlabel('Time (s)')
    plt.ylabel('Gyro (rad/s)')
    plt.legend()
    
    # Plot accelerometer data
    plt.subplot(3, 1, 2)
    plt.plot(timestamps, acc_x, label='Acc X')
    plt.plot(timestamps, acc_y, label='Acc Y')
    plt.plot(timestamps, acc_z, label='Acc Z')
    plt.title('Accelerometer Data')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.legend()
    
    # Plot roll, pitch, yaw
    plt.subplot(3, 1, 3)
    plt.plot(timestamps, roll, label='Roll')
    plt.plot(timestamps, pitch, label='Pitch')
    plt.plot(timestamps, yaw, label='Yaw')
    plt.title('Attitude (Roll, Pitch, Yaw)')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (rad)')
    plt.legend()
    
    plt.tight_layout()
    plt.show()

    # Plot thrust vector data and thrust angle
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(timestamps, thrust_vector_x, label='Thrust Vector X')
    plt.plot(timestamps, thrust_vector_y, label='Thrust Vector Y')
    plt.plot(timestamps, thrust_vector_z, label='Thrust Vector Z')
    plt.title('Thrust Vector Components')
    plt.xlabel('Time (s)')
    plt.ylabel('Thrust Vector')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(timestamps, np.degrees(thrust_angle_from_z), label='Thrust Angle from Z')
    plt.title('Thrust Angle from Z-axis')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Create the monitor and run it
    monitor = HighResPX4Monitor()
    try:
        monitor.run()
    except KeyboardInterrupt:
        # Close the file and plot the data
        print("\nPlotting data...")
        monitor.csv_file.close()
        plot_data(monitor.csv_file.name)
        input("Press Enter to exit...")
