import pandas as pd
count = 0
# Load the CSV into a DataFrame
df = pd.read_csv('DriveUpload/combined_data.csv')

# Determine if each row is identical to the previous row
df['is_repeat'] = df.eq(df.shift()).all(axis=1)
repeat_count = int(df['is_repeat'].sum())

# For convenience, fill the first row's repeat flag as False
df.loc[0, 'is_repeat'] = False

# Save the result to a new CSV
output_path = 'DriveUpload/repeat_detection.csv'
df.to_csv(output_path, index=False)

print("Total repeat rows:", repeat_count)
print(f"Processed {len(df)} rows. Results saved to '{output_path}'.")
print(f"Percent of repeats: {repeat_count/len(df)}")
