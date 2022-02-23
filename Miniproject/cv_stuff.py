import numpy as np
import cv2 as cv
cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
    
    low_yel = np.array([20,100,100])
    up_yel = np.array([30,255,255])
    mask = cv.inRange(hsv,low_yel,up_yel)
    img_msk = cv.bitwise_and(frame,frame,mask= mask)
    
    #img = (img_msk*1.0 / img_msk.mean(axis=(0,1)))
    img = img_msk
    
    #kern = np.ones((5,5),np.float32)/25
    #smooth = cv.filter2D(img_msk,-1,kern)
    #kern = np.ones((5,5),np.uint8)
    #smooth = cv.morphologyEx(cv.morphologyEx(img_msk, cv.MORPH_OPEN, kern), cv.MORPH_CLOSE, kern)
    
    gry = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret,thresh = cv.threshold(gry,50,255,cv.THRESH_BINARY)
    tup = np.nonzero(thresh)
    if not np.any(tup[0]):
        print('No markers found')
    else:
        x_zero = img.shape[1]*0.5
        y_zero = img.shape[0]*0.5
        x_coor = round(np.mean(tup[1]))
        y_coor = round(np.mean(tup[0]))
        quad = 0
        if x_zero < x_coor and y_zero >= y_coor:
            quad = 1
        elif x_zero >= x_coor and y_zero > y_coor:
            quad = 2
        elif x_zero > x_coor and y_zero <= y_coor:
            quad = 3
        elif x_zero <= x_coor and y_zero < y_coor:
            quad = 4
        print('Quadrant: {}'.format(quad))

    # Display the resulting frame
    cv.imshow('frame', img)
    if cv.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv.destroyAllWindows()