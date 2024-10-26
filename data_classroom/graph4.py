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

# Combine all data into one DataFrame and sort by Trial and Goal
all_data = pd.concat([data_1, data_2, data_3, data_4, data_5])
all_data = all_data.sort_values(by=['Trial', 'Goal']).reset_index(drop=True)

# Initialize a DataFrame to hold results with all trials and goals
# Define 5 trials with 4 goals each, filling in NaN if any are missing
trials_goals = pd.MultiIndex.from_product([range(1, 6), range(1, 5)], names=['Trial', 'Goal'])
all_data = all_data.set_index(['Trial', 'Goal']).reindex(trials_goals).reset_index()

# Calculate total path length per trial and organize data for each goal
results = []
for trial, trial_data in all_data.groupby('Trial'):
    # Calculate total path length for the trial, ignoring NaNs
    total_path_length = trial_data['Path Length (m)'].sum()
    
    # Extract path lengths for each goal
    path_length_goal_1 = trial_data.loc[trial_data['Goal'] == 1, 'Path Length (m)'].values[0]
    path_length_goal_2 = trial_data.loc[trial_data['Goal'] == 2, 'Path Length (m)'].values[0]
    path_length_goal_3 = trial_data.loc[trial_data['Goal'] == 3, 'Path Length (m)'].values[0]
    path_length_goal_4 = trial_data.loc[trial_data['Goal'] == 4, 'Path Length (m)'].values[0]
    
    # Store results in a dictionary
    trial_result = {
        'Trial': trial,
        'Total Path Length (m)': total_path_length,
        'Path Length Goal 1 (m)': path_length_goal_1,
        'Path Length Goal 2 (m)': path_length_goal_2,
        'Path Length Goal 3 (m)': path_length_goal_3,
        'Path Length Goal 4 (m)': path_length_goal_4,
    }
    
    # Append the result for this trial
    results.append(trial_result)

# Create a results DataFrame
results_df = pd.DataFrame(results)

# Display the table
print("Path Lengths Summary:")
print(results_df.to_string(index=False))
