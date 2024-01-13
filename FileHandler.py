import re
import csv

def text_file_to_map(file_path):
    dimensions = {}
    try:
        with open(file_path, 'r', encoding='utf-8-sig') as file:
            for line in file:
                # Remove whitespace and check if the line is not empty
                line = line.strip()
                if line:
                    # Using regex to extract dimension name, value, and unit
                    match = re.match(r'"([^"]+)"\s*=\s*([\d.]+)\s*([a-zA-Z]*)', line)
                    if match:
                        dimension_name = match.group(1)
                        dimension_value = float(match.group(2))
                        dimension_unit = match.group(3) or None
                        dimensions[dimension_name] = (dimension_value, dimension_unit)
        return dimensions
    except Exception as e:
        print(f"An error occurred while processing the file: {e}")
        return {}

def csv_to_map(file_path):
    dimensions = {}
    with open(file_path, newline='', encoding='utf-8-sig') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            if row and len(row) >= 3:
                # Extracting dimension name, value, and unit
                key = row[0]
                try:
                    value = float(row[1])
                except ValueError:
                    # Handle the case where the value is not a valid float
                    continue
                unit = row[2] if row[2] else None
                dimensions[key] = (value, unit)
    return dimensions

def map_to_text_file(dimensions, file_path):
    try:
        with open(file_path, 'w', encoding='utf-8-sig') as file:
            for dimension_name, (dimension_value, dimension_unit) in dimensions.items():
                # Handling the case where the dimension is unitless
                unit_str = dimension_unit if dimension_unit != None else ''
                file.write(f'"{dimension_name}"= {dimension_value}{unit_str}\n\n')
    except Exception as e:
        print(f"An error occurred while writing to the file: {e}")