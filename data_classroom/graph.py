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
all_data = pd.concat([data_1, data_2, data_3])#,, data_4 data_5])

# Group by Trial and calculate the mean path length for each trial
mean_path_lengths = all_data.groupby('Trial')['Path Length (m)'].mean()

# Plot the mean path length for each trial
plt.figure(figsize=(8, 6))
mean_path_lengths.plot(kind='bar', color='skyblue', edgecolor='black')
plt.title('Average Path Length for Each Trial')
plt.xlabel('Trial')
plt.ylabel('Mean Path Length (m)')
plt.xticks(rotation=0)
plt.tight_layout()
plt.show()
