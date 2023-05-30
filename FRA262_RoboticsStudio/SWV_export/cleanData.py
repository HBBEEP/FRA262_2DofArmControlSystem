import csv

input_file = 'duty500New.txt'
output_file = 'duty500Clean.csv'


def clean_data(input_data):
    try:
        # Remove double quotes
        cleaned_data = input_data.replace('"', '')

        # Split the cleaned data into individual elements
        elements = cleaned_data.split(';')

        # Check if the row has the expected number of elements
        if len(elements) != 5:
            return None

        # Convert numeric values to appropriate types
        elements[1] = float(elements[1]) if elements[1] != '' else None
        elements[2] = int(elements[2]) if elements[2] != '' else None
        time_value = elements[3].split()[0] if elements[3] != '' else None
        elements[3] = float(time_value.strip('s')) if time_value is not None else None

        return elements
    except Exception as e:
        print(f"Error cleaning data: {e}")
        return None

# Read input data from text file
with open(input_file, 'r') as file:
    input_data = file.readlines()

# Clean the data
cleaned_data = []
for row in input_data:
    cleaned_row = clean_data(row.strip())
    if cleaned_row is not None and cleaned_row[1] != 0.0:
        cleaned_data.append(cleaned_row)

# Write cleaned data to CSV file
with open(output_file, 'w', newline='') as file:
    writer = csv.writer(file)
    for row in cleaned_data:
        writer.writerow([elem.replace('s', '') if isinstance(elem, str) else elem for elem in row])

print("Data cleaned and stored in", output_file)
