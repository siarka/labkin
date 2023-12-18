import cv2
# Ustaw parametry zapisu wideo
#fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Wybierz kodek kompresji (tutaj używamy XVID)
#out = cv2.VideoWriter('zapisany_film.mp4', fourcc, 20.0, (640, 480))  # Ustaw nazwę pliku, kodek, liczbę klatek na sekundę (fps) i rozdzielczość
# Otwórz kamerę (numer kamery 0 oznacza kamerę domyślną)
cap = cv2.VideoCapture("kod.mp4")

#cap = cv2.VideoCapture('rtsp://username:password@192.168.1.64/1')
#cap = cv2.VideoCapture('rtsp://192.168.1.64/1')
#cap = cv2.VideoCapture(192.168.5.31)
#cap = cv2.VideoCapture(192.168.5.31/camera)
#cap = cv2.VideoCapture.open(192.168.5.31)
#cap = cv2.VideoCapture.open(192.168.5.31/camera)
#cap = cv2.VideoCapture( )
while True:
    ret, frame = cap.read()
    if not ret:
        break
    cv2.imshow("Kamera", frame)
 #   out.write(frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
#out.release()
cap.release()
cv2.destroyAllWindows()


