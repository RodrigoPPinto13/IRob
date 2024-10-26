import pandas as pd

# Load the CSV files
file_1 = 'path_to_file/move_base_performance_1.csv'
file_2 = 'path_to_file/move_base_performance_2.csv'
file_4 = 'path_to_file/move_base_performance_4.csv'

# Read the CSV files
data_1 = pd.read_csv(file_1)
data_2 = pd.read_csv(file_2)
data_4 = pd.read_csv(file_4)

# Combine the data into one DataFrame and sort by trial to ensure order
all_data = pd.concat([data_1, data_2, data_4]).sort_values(by=['Trial'])

# Initialize lists to store results
results = []

# Calculate path lengths
for trial, trial_data in all_data.groupby('Trial'):
    # Total path length for the trial
    total_path_length = trial_data['Path Length (m)'].sum()
    
    # Path length between each goal (goal 1-2, 2-3, 3-4)
    path_lengths_between_goals = trial_data['Path Length (m)'].diff().dropna().tolist()
    
    # Create dictionary to store values for each trial
    trial_result = {
        'Trial': trial,
        'Total Path Length (m)': total_path_length,
        'Path Length 1-2 (m)': path_lengths_between_goals[0] if len(path_lengths_between_goals) > 0 else None,
        'Path Length 2-3 (m)': path_lengths_between_goals[1] if len(path_lengths_between_goals) > 1 else None,
        'Path Length 3-4 (m)': path_lengths_between_goals[2] if len(path_lengths_between_goals) > 2 else None,
    }
    
    # Append trial result to results list
    results.append(trial_result)

# Create DataFrame from results
results_df = pd.DataFrame(results)

# Display the table
print("Path Lengths Summary:")
print(results_df.to_string(index=False))
