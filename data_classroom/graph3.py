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

# Combine all data into one DataFrame and sort by Trial
all_data = pd.concat([data_1, data_2, data_3, data_4, data_5]).sort_values(by=['Trial'])

# Initialize list to store results
results = []

# Loop through each trial and prepare values
for trial, trial_data in all_data.groupby('Trial'):
    # Calculate total path length for the trial
    total_path_length = trial_data['Path Length (m)'].sum()
    
    # Get path length for each goal in the trial (Goal 1, Goal 2, Goal 3, and Goal 4)
    path_lengths = trial_data['Path Length (m)'].tolist()
    
    # Ensure we have exactly 4 goals per trial, filling missing ones with None
    path_lengths += [None] * (4 - len(path_lengths))
    
    # Store trial information in a dictionary
    trial_result = {
        'Trial': trial,
        'Total Path Length (m)': total_path_length,
        'Path Length Goal 1 (m)': path_lengths[0],
        'Path Length Goal 2 (m)': path_lengths[1],
        'Path Length Goal 3 (m)': path_lengths[2],
        'Path Length Goal 4 (m)': path_lengths[3],
    }
    
    # Append trial result to results list
    results.append(trial_result)

# Create DataFrame from results
results_df = pd.DataFrame(results)

# Display the table
print("Path Lengths Summary:")
print(results_df.to_string(index=False))
