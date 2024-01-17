import matplotlib.pyplot as plt
import pandas as pd


csv_file_path = r'C:\Users\artur\Downloads\epuck_crazyflie\robot_trajectory.csv'
df = pd.read_csv(csv_file_path)

x_values = df['X']
y_values = df['Y']

plt.plot(x_values, y_values, marker='o', linestyle='-', color='b', label='Path')

plt.xlabel('X Axis Label')
plt.ylabel('Y Axis Label')
plt.title('Epuck Path')

plt.legend()

plt.show()
