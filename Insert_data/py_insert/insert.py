import pyodbc
import serial
from datetime import datetime

server = 'Felix-Nguyen\SQLEXPRESS'
database = 'final_project_microcontroller'


conn = pyodbc.connect(
    f'DRIVER={{ODBC Driver 17 for SQL Server}};SERVER={server};DATABASE={database};Trusted_Connection=yes;'
)
cursor = conn.cursor()

ser = serial.Serial('COM7', 9600, timeout=1) 

print("Listening for data...")

while True:
    try:
        data = ser.readline().decode().strip()  
        if data:
            reading_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            temperature = float(data)
            cursor.execute("INSERT INTO dbo.temperature (reading_time, temperature_value) VALUES (?, ?)",
    (reading_time, temperature))
            conn.commit()
            print(f"Inserted reading time: {reading_time}")
            print(f"Inserted temperature: {temperature}")

    except Exception as e:
        print(f"Error: {e}")
        break

cursor.close()
conn.close()
ser.close()