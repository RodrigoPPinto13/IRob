import pandas as pd

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

# Combine the data into one DataFrame and sort by trial to ensure order
all_data = pd.concat([data_1, data_2, data_3, data_4, data_5]).sort_values(by=['Trial'])

# Initialize lists to store results
results = []

# Calculate path lengths
for trial, trial_data in all_data.groupby('Trial'):
    # Total path length for the trial
    total_path_length = trial_data['Path Length (m)'].sum()
    
    # Path length for each goal directly
    path_length_goal_1 = trial_data.iloc[0]['Path Length (m)'] if len(trial_data) > 0 else None
    path_length_goal_2 = trial_data.iloc[1]['Path Length (m)'] if len(trial_data) > 1 else None
    path_length_goal_3 = trial_data.iloc[2]['Path Length (m)'] if len(trial_data) > 2 else None
    path_length_goal_4 = trial_data.iloc[3]['Path Length (m)'] if len(trial_data) > 3 else None
    
    # Create dictionary to store values for each trial
    trial_result = {
        'Trial': trial,
        'Total Path Length (m)': total_path_length,
        'Path Length Goal 1 (m)': path_length_goal_1,
        'Path Length Goal 2 (m)': path_length_goal_2,
        'Path Length Goal 3 (m)': path_length_goal_3,
        'Path Length Goal 4 (m)': path_length_goal_4,
    }
    
    # Append trial result to results list
    results.append(trial_result)

# Create DataFrame from results
results_df = pd.DataFrame(results)

# Display the table
print("Path Lengths Summary:")
print(results_df.to_string(index=False))
