import pandas as pd

# Read the CSV file
df = pd.read_csv('scriptyQuat.csv')

# Group the data based on the first column
grouped = df.groupby('Column 0')

# Iterate over the groups and display the data
for name, group in grouped:
    print(f"Group {name}:")
    print(group)
    print()  # Add an empty line between groups
