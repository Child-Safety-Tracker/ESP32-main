import json
import csv

# Đọc file JSON
with open('formatted_filtered_response_output.json', 'r') as json_file:
    data = json.load(json_file)

# Mở file CSV để ghi
with open('data.csv', 'w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    
    # Ghi tiêu đề cột
    header = ['Date', 'Time']
    csv_writer.writerow(header)
    
    # Ghi dữ liệu
    for entry in data:
        date = entry['date']
        times = entry['times']
        for time in times:
            csv_writer.writerow([date, time])