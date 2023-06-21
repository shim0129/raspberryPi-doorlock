import RPi.GPIO as GPIO
import time
import cv2

TRIG_PIN = 23  
ECHO_PIN = 24
LED_PIN = 18  

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)
GPIO.setup(LED_PIN, GPIO.OUT)

cap = cv2.VideoCapture(0)

# 출입한 사람 수 변수
people_count = 0

def get_distance():
    GPIO.output(TRIG_PIN, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    # Echo 핀으로 돌아오는 초음파 신호 시간 측정
    pulse_start = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.LOW:
        pulse_start = time.time()

    pulse_end = time.time()
    while GPIO.input(ECHO_PIN) == GPIO.HIGH:
        pulse_end = time.time()

    distance = (pulse_end - pulse_start) * 343 / 2

    return distance

try:
    while True:
        distance = get_distance()
        if distance < 20:
            GPIO.output(LED_PIN, GPIO.HIGH)
            ret, frame = cap.read()
            if not ret:
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
            detections = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            for (x, y, w, h) in detections:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            if len(detections) > 0:
                people_count += 1
                cv2.putText(frame, "People Count: {}".format(people_count), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow("Frame", frame)
            if cv2.waitKey(0) == ord('s'):
            # 파일명 지정
                filename = "captured_image.jpg"

            # 프레임을 이미지 파일로 저장
                cv2.imwrite(filename, frame)
                print("이미지가 저장되었습니다.")
        else:
            GPIO.output(LED_PIN, GPIO.LOW)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
