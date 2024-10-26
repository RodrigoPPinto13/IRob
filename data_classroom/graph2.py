import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV files
file_1 = 'move_base_performance_1.csv'
file_2 = 'move_base_performance_2.csv'
file_3 = 'move_base_performance_3.csv'
file_4 = 'move_base_performance_4.csv'
file_5 = 'move_base_performance_5.csv'

# Read the CSV files
data_1 = pd.read_csv(file_1)
data_2 = pd.read_csv(file_2)
data_3 = pd.read_csv(file_3)
data_4 = pd.read_csv(file_4)
data_5 = pd.read_csv(file_5)

# Combine the data into one DataFrame
all_data = pd.concat([data_1, data_2, data_3, data_4, data_5])

# Calculate speed (Path Length / Time to Goal) for each trial
# Convert the 'Time to Goal (s)' from Unix timestamp to seconds
# We assume that 'Time to Goal (s)' is already in seconds
all_data['Speed (m/s)'] = all_data['Path Length (m)'] / all_data['Time to Goal (s)']

# Group by Trial and calculate the mean speed for each trial
mean_speeds = all_data.groupby('Trial')['Speed (m/s)'].mean()

# Plot the mean speed for each trial
plt.figure(figsize=(8, 6))
mean_speeds.plot(kind='bar', color='lightgreen', edgecolor='black')
plt.title('Average Speed for Each Trial')
plt.xlabel('Goal')
plt.ylabel('Mean Speed (m/s)')
plt.xticks(rotation=0)
plt.tight_layout()
plt.show()
