import socket

# Parametry komunikacji UDP
udp_host = "localhost"  # Adres hosta, z którego odbieramy dane
udp_port = 12345  # Port, na którym odbieramy dane
buffer_size = 1024  # Rozmiar bufora odbiorczego

# Inicjalizacja gniazda UDP
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind((udp_host, udp_port))

# Nazwa pliku do zapisu danych
output_file = "received_waypoints.txt"

while True:
    try:
        data, addr = udp_socket.recvfrom(buffer_size)
        data = data.decode()  # Dekoduj dane z bajtów do tekstu

        # Podziel otrzymane dane na pojedyncze liczby
        received_numbers = list(map(float, data.split()))

        # Zapisz otrzymane liczby do pliku
        with open(output_file, "a") as file:
            file.write(" ".join(map(str, received_numbers)) + "\n")

        print(f"Received and saved data: {received_numbers}")
    except KeyboardInterrupt:
        # Obsługa przerwania przez użytkownika (Ctrl+C)
        print("Received Ctrl+C. Exiting.")
        break
    except Exception as e:
        print(f"Error: {e}")

udp_socket.close()
