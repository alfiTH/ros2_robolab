#!/usr/bin/env python3

import subprocess
import time
import pandas as pd
import os

# Duración de la medición para cada tópico (en segundos)
MEASUREMENT_DURATION = 5

# Lista para almacenar los datos en el DataFrame
topic_data = pd.DataFrame(columns=["Topic", "Average Rate (Hz)"])

def get_topic_list():
    """Obtiene la lista de tópicos activos en ROS 2."""
    try:
        topics = subprocess.check_output(["ros2", "topic", "list"]).decode().splitlines()
        return topics
    except subprocess.CalledProcessError as e:
        print(f"Error al obtener la lista de tópicos: {e}")
        return []

def measure_topic_rate(topic):
    """Mide la frecuencia promedio de un tópico."""
    try:
        # Ejecuta ros2 topic hz y captura la salida
        process = subprocess.Popen(
            ["ros2", "topic", "hz", topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        time.sleep(MEASUREMENT_DURATION)  # Espera para obtener una muestra
        process.terminate()  # Detiene el comando ros2 topic hz
        stdout, stderr = process.communicate()
        rate = None

        # Procesa la salida para extraer la frecuencia promedio
        for line in stdout.splitlines():
            if "average rate:" in line:
                rate = float(line.split("average rate:")[1].split()[0])
        return rate
        
    except Exception as e:
        print(f"Error al medir la frecuencia de {topic}: {e}")
        return None

def update_dataframe(topic, rate):
    """Actualiza el DataFrame con la medición de la frecuencia."""
    global topic_data
    new_data = pd.DataFrame({"Topic": [topic], "Average Rate (Hz)": [rate]})
    topic_data = pd.concat([topic_data, new_data], ignore_index=True)
    print(topic_data)  # Imprimir el DataFrame actualizado

def main():
    # Obtener la lista de tópicos
    topics = get_topic_list()
    if not topics:
        print("No se encontraron tópicos activos.")
        return

    # Medir la frecuencia de cada tópico y actualizar el DataFrame
    for topic in topics:
        print(f"Midiendo la frecuencia de {topic}...")
        rate = measure_topic_rate(topic)
        if rate is not None:
            print(f"Frecuencia promedio de {topic}: {rate} Hz")
            update_dataframe(topic, rate)
        else:
            print(f"No se pudo medir la frecuencia de {topic}")
    topic_data.to_csv("rates.csv", index=False)

if __name__ == "__main__":
    main()

