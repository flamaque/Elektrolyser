import mysql.connector
from mysql.connector import errorcode
from flask import Flask, request, jsonify
from datetime import datetime
import socket
import ssl

app = Flask(__name__)
DB_CONFIG = {
    'host': '127.0.0.1',
    'user': 'pee51',
    'password': 'Wachtwoord',
    'database': 'electrolyser',
    'ssl_cert': '/home/jochem/Documents/pee51_client-certificate.pem',
    'ssl_key': '/home/jochem/Documents/pee51_client-key.pem',
    'ssl_ca': '/home/jochem/Documents/ca_certificate.pem'
}

def get_db_connection():
    connection = getattr(Flask, '_connection', None)
    if connection is None:
        connection = mysql.connector.connect(**DB_CONFIG)
        Flask._connection = connection
    return connection

def close_db_connection():
    connection = getattr(Flask, '_connection', None)
    if connection is not None:
        connection.close()
        del Flask._connection

def insert_telemetry_data(data):
    connection = None
    cursor = None
    try:
        connection = get_db_connection()
        cursor = connection.cursor()

        # Define possible keys
        possible_keys = ['T_g', 'pH', 'A', 'V', 'P', 'T1', 'T2', 'T3', 'T4', 'T5', 'Hum', 'Cond', 'Flow1', 'H2', 'FT', 'ts']

        for entry in data:
            # Extract the timestamp
            ts = entry.get('ts', None)
            values = entry.get('values', {})

            # Create the list of values, including the timestamp
            data_tuple = tuple(values.get(key, None) for key in possible_keys[:-1]) + (ts,)

            # Create the query dynamically based on the keys present
            columns = ", ".join(possible_keys)
            placeholders = ", ".join(["%s"] * len(possible_keys))
            query = f"INSERT INTO Electrolyser ({columns}) VALUES ({placeholders})"

            cursor.execute(query, data_tuple)
            connection.commit()

        print("Telemetry data inserted successfully")
    except Exception as e:
        print("Error inserting telemetry data:", e)
    finally:
        if cursor:
            cursor.close()
        if connection:
            connection.close()

@app.route('/api/telemetry', methods=['POST'])
def log_telemetry():
    try:
        print("Received POST request")
        data = request.get_json()
        if data:
            print("Data:", data)
            insert_telemetry_data(data)
            return jsonify({"message": "Telemetry logged successfully"}), 200
        else:
            raise ValueError("No data received")
    except Exception as e:
        print("Error processing request:", e)
        return jsonify({"error": str(e)}), 500

@app.teardown_appcontext
def teardown_db(exception):
    close_db_connection()

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)