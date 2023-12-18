import cv2
import time

# Wczytaj obraz z pliku
image = cv2.imread("FLIR0025.jpg")

# Wyświetl obraz
cv2.imshow("Kamera", image)


warunek = True

while warunek:
    key = cv2.waitKey(1)

    print("Naciśnij w, s, a, d lub q:")

    if key == ord('w'):
        print("Wciśnięto 'w'")
        time.sleep(1)
    elif key == ord('s'):
        print("Wciśnięto 's'")
        time.sleep(1)
    elif key == ord('a'):
        print("Wciśnięto 'a'")
        time.sleep(1)
    elif key == ord('d'):
        print("Wciśnięto 'd'")
        time.sleep(1)
    elif key == ord('q'):
        warunek = False
        cv2.destroyAllWindows()


