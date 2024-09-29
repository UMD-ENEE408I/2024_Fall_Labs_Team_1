import cv2

cam = cv2.VideoCapture('/dev/video0') 
result, image = cam.read() 

if result:
	gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
	faces_rect = face_cascade.detectMultiScale(gray_image, 1.1, 9) 

	for (x, y, w, h) in faces_rect: 
	    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), thickness=2) 
	    
	cv2.imshow("image", image)
	cv2.imwrite("detected.png", image)
	cv2.waitKey(0)
	cv2.destroyWindow("image") 
else: 
    print("No image detected.") 
