# Read the file
with open("smoothed_inside_line_copy.csv", "r") as file:
    lines = file.readlines()

# Keep every other line
filtered_lines = lines[::2]

# Write back to a new file or overwrite the original file
with open("output_file.txt", "w") as file:
    file.writelines(filtered_lines)