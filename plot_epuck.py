import matplotlib.pyplot as plt
import pandas as pd


# Load the CSV file into a DataFrame
csv_file_path = r'C:\Users\artur\Downloads\epuck\output_positions.csv'
df = pd.read_csv(csv_file_path)

# Extract x and y values from the DataFrame
x_values = df['X']
y_values = df['Y']

# Plot the x and y values
plt.plot(x_values, y_values, marker='o', linestyle='-', color='b', label='Path')

# Add labels and title
plt.xlabel('X Axis Label')
plt.ylabel('Y Axis Label')
plt.title('Epuck Path')

# Add a legend
plt.legend()

# Show the plot
plt.show()
