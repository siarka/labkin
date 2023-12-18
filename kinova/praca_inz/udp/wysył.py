import math
import random
import time
import socket

# Parametry generowania waypointów
radius = 0.5
center_x = 0.0
center_y = 0.0
orientation_range = (-10, 10)

# Parametry komunikacji UDP
udp_host = "localhost"  # Adres docelowy hosta
udp_port = 12345  # Port docelowy

# Inicjalizacja gniazda UDP
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    # Generuj pojedynczy waypoint
    angle = (2 * math.pi * time.time())  # Wykorzystaj aktualny czas do generowania waypointu na okręgu
    x = round(center_x + radius * math.cos(angle), 3)
    y = round(center_y + radius * math.sin(angle), 3)
    z = round(0.0, 3)
    blending_radius = round(0.05, 3)
    theta_x = round(random.uniform(*orientation_range), 3)
    theta_y = round(random.uniform(*orientation_range), 3)
    theta_z = round(random.uniform(*orientation_range), 3)

    waypoint = f"{x} {y} {z} {blending_radius} {theta_x} {theta_y} {theta_z}\n"

    # Wysyłanie waypointu do hosta i portu za pomocą protokołu UDP
    udp_socket.sendto(waypoint.encode(), (udp_host, udp_port))

    print(f"Generated waypoint and sent it via UDP.")
    time.sleep(1)  # Czekaj 1 sekundę przed wygenerowaniem kolejnego waypointu
