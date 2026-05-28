import cv2
import numpy as np
import math
from picamera2 import Picamera2

THRESHOLD_VALUE = 200   
MIN_AREA = 9              
MAX_AREA = 400             
MERGE_RADIUS = 50          
RESOLUTION_W = 1280        
RESOLUTION_H = 800         

detected_targets = []
target_id_counter = 0

def process_frame(frame):
    global detected_targets, target_id_counter
    
  
    gray = frame[:RESOLUTION_H, :]

    _, thresh = cv2.threshold(gray, THRESHOLD_VALUE, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    current_frame_centroids = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if MIN_AREA < area < MAX_AREA:
            # Momenty obrazu pozwalają wyznaczyć matematyczny środek plamki
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                current_frame_centroids.append((cx, cy))

    for target in detected_targets:
        target['frames_unseen'] += 1

    for (cx, cy) in current_frame_centroids:
        matched = False
        for target in detected_targets:
            dist = math.hypot(target['x'] - cx, target['y'] - cy)
            
            if dist < MERGE_RADIUS:
                target['x'] = cx  # Aktualizujemy jej bieżącą pozycję na ekranie
                target['y'] = cy
                target['frames_unseen'] = 0 # Zerujemy licznik, bo dioda znów świeci
                matched = True
                break
        
        if not matched:
            detected_targets.append({
                'id': target_id_counter,
                'x': cx,
                'y': cy,
                'frames_unseen': 0
            })
            target_id_counter += 1

    display_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    
    for target in detected_targets:
        if target['frames_unseen'] < 60:
            cv2.circle(display_img, (target['x'], target['y']), MERGE_RADIUS, (0, 255, 0), 1)
            cv2.putText(display_img, f"ID:{target['id']}", (target['x'] + 10, target['y'] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.drawMarker(display_img, (target['x'], target['y']), (0, 0, 255), cv2.MARKER_CROSS, 10, 2)

    return display_img

if __name__ == "__main__":
    # Inicjalizacja kamery
    picam2 = Picamera2()
    
    config = picam2.create_video_configuration(
        {"main": {"format": "YUV420", "size": (RESOLUTION_W, RESOLUTION_H)}}
    )
    picam2.configure(config)
    picam2.start()
    
    print("System wizyjny uruchomiony. Trwa skanowanie w poszukiwaniu modułów OOK...")
    print("Wciśnij 'q' na klawiaturze (przy aktywnym oknie kamery), aby zakończyć.")
    
    try:
        while True:
            frame = picam2.capture_array()
            
            output = process_frame(frame)
            
            cv2.imshow("Detekcja OOK (RPi5 + OV9281)", output)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("\nPrzerwano działanie programu kombinacją klawiszy (Ctrl+C).")
        
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        
        print("\n===================")
        print("Ostateczna lista zlokalizowanych diod:")
        print("=====================")
        if not detected_targets:
            print("Nie wykryto żadnych diod podczas tej sesji.")
        else:
            for t in detected_targets:
                print(f"Moduł ID: {t['id']} | Ostatnia znana pozycja -> X: {t['x']}, Y: {t['y']}")