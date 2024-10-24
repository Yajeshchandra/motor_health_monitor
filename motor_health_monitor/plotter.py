import matplotlib.pyplot as plt
import numpy as np
import csv


def plot_data(csv_filename):
    # Load the CSV data
    timestamps = []
    gyro_x, gyro_y, gyro_z = [], [], []
    acc_x, acc_y, acc_z = [], [], []
    roll, pitch, yaw = [], [], []
    thrust_vector_x, thrust_vector_y, thrust_vector_z = [], [], []
    thrust_angle_from_z = []
    motor1,motor2,motor3,motor4=[],[],[],[]
    
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
            motor1.append(float(row['Servo1_Raw']))
            motor2.append(float(row['Servo2_Raw']))
            motor3.append(float(row['Servo3_Raw']))
            motor4.append(float(row['Servo4_Raw']))
    
    # Convert timestamps to seconds
    timestamps = np.array(timestamps) / 1e9  # Convert from ns to s
    # gyro_x=gyro_x[10940:11200]
    # gyro_y=gyro_y[10940:11200]
    # gyro_z=gyro_z[10940:11200]
    # acc_x=acc_x[10940:11200]
    # acc_y=acc_y[10940:11200]
    # acc_z=acc_z[10940:11200]
    # roll=roll[10940:11200]
    # pitch=pitch[10940:11200]
    # yaw=yaw[10940:11200]
    # thrust_vector_x=thrust_vector_x[10940:11200]
    # thrust_vector_y=thrust_vector_y[10940:11200]
    # thrust_vector_z=thrust_vector_z[10940:11200]
    # thrust_angle_from_z=thrust_angle_from_z[10940:11200]
    # timestamps=timestamps[10940:11200]
    
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
    
    # Plot motor data
    plt.figure(figsize=(10, 10))
    
    plt.subplot(2, 2, 1)
    plt.plot(timestamps, motor1, label='Motor 1')
    plt.title('Motor 1')
    plt.xlabel('Time (s)')
    plt.ylabel('Servo1 Raw')
    plt.legend()
    
    plt.subplot(2, 2, 2)
    plt.plot(timestamps, motor2, label='Motor 2')
    plt.title('Motor 2')
    plt.xlabel('Time (s)')
    plt.ylabel('Servo2 Raw')
    plt.legend()
    
    plt.subplot(2, 2, 3)
    plt.plot(timestamps, motor3, label='Motor 3')
    plt.title('Motor 3')
    plt.xlabel('Time (s)')
    plt.ylabel('Servo3 Raw')
    plt.legend()
    
    plt.subplot(2, 2, 4)
    plt.plot(timestamps, motor4, label='Motor 4')
    plt.title('Motor 4')
    plt.xlabel('Time (s)')
    plt.ylabel('Servo4 Raw')
    plt.legend()
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":

    plot_data('px4_data.csv')
