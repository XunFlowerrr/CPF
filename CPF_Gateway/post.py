import requests
import json

# URL of the PHP script
url = 'https://c789-2403-6200-89a7-f25-f436-a043-7d99-6a6a.ngrok-free.app/Dashboard/api/update_data.php'  # Replace with your actual URL

# Data to send in the POST request
data = {
    "sensor_id": 123,
    "gateway_id": "GATEWAY001",
    "data_kwh": 15.75
}

# Convert the data dictionary to JSON
headers = {'Content-Type': 'application/json'}
response = requests.post(url, data=json.dumps(data), headers=headers)

# Check the response status
if response.status_code == 200:
    try:
        response_data = response.json()  # Convert the JSON response to a dictionary
        print("Response:", response_data)
    except json.JSONDecodeError:
        print("Error decoding JSON response.")
else:
    print(f"Failed to send data. Status code: {response.status_code}")

