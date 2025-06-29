To import OpenCV in python:

```python
import cv2 as cv
```

### Reading images and videos:
---
when we read a image it takes the path and returns it as a matrix of pixels

```python
import cv2 as cv

img = cv.imread('/home/vedant/OpenCV/Learning_OpenCV/cat.jpg')

img = cv.resize(img, (640,427))

cv.imshow('Cat',img) #window name,image

cv.waitKey(0) #dosnt close automatically
```

when video is to be read we use the VideoCapture function, for webcam you input the integer value of 0, you use a while loop to read the video frame by frame, can be closed when press 'd'

```python
import cv2 as cv

capture = cv.VideoCapture('/home/vedant/OpenCV/Learning_OpenCV/dog.mp4')

while True:
	isTrue, frame = capture.read() #returns frames and isTrue value which checks if frame was successfully read
	cv.imshow('Video' , frame)
	if cv.waitKey(20) & 0xFF==ord('d'):
		break

capture.release()

cv.destroyAllWindows()
```

when resizing an video/images/live video we have can define a function which inputs the frames and multiplies both height and width by a scale

```python
import cv2 as cv

def rescaleFrame(frame, scale=0.75):

	width = int(frame.shape[1] * scale) #shape[1] is width of frame
	
	height = int(frame.shape[0] * scale) #shape[0] is heigth
	
	dimension = (width,height)

	return cv.resize(frame, dimension, interpolation=cv.INTER_AREA)
	
capture = cv.VideoCapture('/home/vedant/OpenCV/Learning_OpenCV/dog.mp4')

while True:

	isTrue, frame = capture.read() #returns frames and isTrue value which checks if frame was successfully read
	
	frame_resized = rescaleFrame(frame)
	
	cv.imshow('Video' , frame)
	
	cv.imshow('Video_Resized' , frame_resized)
	
	if cv.waitKey(20) & 0xFF==ord('d'):
	
		break


capture.release()
cv.destroyAllWindows()
```

for resize live videos only we use this function:
```python
def changeRes(width,height):
	capture.set(3,width)
	capture.set(4,height)
```

### Drawing on images:
---
create a numpy zeroes and give it 3 values (height, width, number of channels) 

```python
import cv2 as cv
import numpy as np

blank = np.zeros((500,500,3), dtype='uint8') #blank image

# 1. painting image a color
blank[:] = 0,100,200 #referencing all pixels to green

cv.imshow("Blank", blank)
cv.waitKey(0)
```

 ![[Screenshot from 2025-06-26 23-20-18.png]]
```python
import cv2 as cv
import numpy as np

blank = np.zeros((500,500,3), dtype='uint8') #blank image
# 1. painting image a color

blank[200:300, 300:400] = 0,255,0 #referencing all pixels to green
cv.imshow("Blank", blank)
cv.waitKey(0)
```

![[Screenshot from 2025-06-26 23-23-05.png]]
to draw a rectangle:

```python
import cv2 as cv
import numpy as np

blank = np.zeros((500,500,3), dtype='uint8') #blank image

# 2. draw rectangle
cv.rectangle(blank, (100,100), (250,250), (0,250,0),thickness=2)
cv.imshow('Rectangle',blank)
cv.waitKey(0)
```
![[Screenshot from 2025-06-26 23-27-07.png]]

to fill the rectangle:
```python
cv.rectangle(blank, (100,100), (250,250), (0,250,0),thickness=cv.FILLED)
```

to draw a circle:

```python
import cv2 as cv
import numpy as np
blank = np.zeros((500,500,3), dtype='uint8') #blank image

#3. draw circle
cv.circle(blank, (250,250), 40, (0,0,255), thickness=3 )
cv.imshow('Circle',blank)
cv.waitKey(0)
```

to draw line:
```python
import cv2 as cv
import numpy as np

blank = np.zeros((500,500,3), dtype='uint8') #blank image

#3. draw circle
cv.line(blank, (100,100), (250,250), (0,0,255), thickness=3 )
cv.imshow('Line',blank)
cv.waitKey(0)

```

to write text:

```python
import cv2 as cv
import numpy as np

blank = np.zeros((500,500,3), dtype='uint8') #blank image
#3. write text
cv.putText(blank, 'Hello', (225,225), cv.FONT_HERSHEY_TRIPLEX, 1.0, (0,0,255), thickness=3 )
cv.imshow('Text',blank)
cv.waitKey(0)
```

### Functions in OpenCV
---
Converting to grey scale:

```python
import cv2 as cv

img = cv.imread('/home/vedant/OpenCV/Learning_OpenCV/cat2.jpg')
cv.imshow('Cat', img)
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
cv.imshow('Gray', gray)
cv.waitKey(0)
```

Bluring an image :
to increase blur increase kernal (3,3) size
```python
import cv2 as cv
img = cv.imread('/home/vedant/OpenCV/Learning_OpenCV/cat2.jpg')

cv.imshow('Cat', img)
#Blur to reduce noise

  

blur = cv.GaussianBlur(img, (3,3), cv.BORDER_DEFAULT)

cv.imshow('Blue', blur)

cv.waitKey(0)
```

Edge Cascade:
to reduce the amount of edges just blur the image
```python
import 
img = cv.imread('/home/vedant/OpenCV/Learning_OpenCV/cat2.jpg')
cv.imshow('Cat', img)

#Edge Cascade

canny = cv.Canny(img,125,175)
cv.imshow('Canny',canny)
cv.waitKey(0)
```

Dilating the image:

```python
import cv2 as cv

img = cv.imread('/home/vedant/OpenCV/Learning_OpenCV/cat2.jpg')
cv.imshow('Cat', img)
#Edge Cascade

canny = cv.Canny(img,125,175)
cv.imshow('Canny',canny)

#dilating

dilate = cv.dilate(canny,(7,7),iterations= 1)
cv.imshow('Dialated', dilate)

cv.waitKey(0)
```

Eroding:
to get the edges back from dilating

```python
import cv2 as cv

img = cv.imread('/home/vedant/OpenCV/Learning_OpenCV/cat2.jpg')
cv.imshow('Cat', img)
#Edge Cascade
canny = cv.Canny(img,125,175)
cv.imshow('Canny',canny)
#dilating
dilate = cv.dilate(canny,(7,7),iterations= 1)
cv.imshow('Dialated', dilate)
erode = cv.erode(dilate, (3,3), iterations=1)

cv.imshow('Eroded', erode)

cv.waitKey(0)
```

Resizing Image:
```python
import cv2 as cv

  

img = cv.imread('/home/vedant/OpenCV/Learning_OpenCV/cat2.jpg')

cv.imshow('Cat', img)

  

resize = cv.resize(img, (500,500), interpolation=cv.INTER_AREA)

cv.imshow('Resize', resize)

cv.waitKey(0)
```

Cropping:

```python
import cv2 as cv

  

img = cv.imread('/home/vedant/OpenCV/Learning_OpenCV/cat2.jpg')

cv.imshow('Cat', img)

  

cropped = img[50:200, 200:400]

cv.imshow('Cropped' , cropped)

cv.waitKey(0)

```
### Image Transformations
---
Translation:
-x -> left
-y -> up
+x -> right
+y -> down

```python
import cv2 as cv

import numpy as np

  

img = cv.imread('/home/vedant/OpenCV/Learning_OpenCV/cat2.jpg')

cv.imshow('Cat', img)

  

def translate(img, x, y):

	transMat = np.float32([[1,0,x],[0,1,y]])
	
	dimensions = (img.shape[1],img.shape[0])
	
	return cv.warpAffine(img, transMat, dimensions)

  

translated = translate(img,100,100)

cv.imshow('Trnaslated' , translated)

cv.waitKey(0)
```
Rotation:

```python
import cv2 as cv

  

img = cv.imread('/home/vedant/OpenCV/Learning_OpenCV/cat2.jpg')

cv.imshow('Cat', img)

  

def rotate(img, angle, pt = None):

	(height,width) = img.shape[:2]
	
	  
	
	if pt is None:
	
		pt = (width//2, height//2)
		
	rotMat = cv.getRotationMatrix2D(pt, angle, 1.0)
	
	dimensions = (width,height)
	
	return cv.warpAffine(img, rotMat, dimensions)

  

rotated = rotate(img, 45)

cv.imshow('Rotated', rotated)

cv.waitKey(0)
```
Flipping:
0 -> vertically 
1 -> horizontally 
-1 -> both

```python
import cv2 as cv

  

img = cv.imread('/home/vedant/OpenCV/Learning_OpenCV/cat2.jpg')

cv.imshow('Cat', img)

  

flipped = cv.flip(img, 1)

cv.imshow("Flip", flipped)

cv.waitKey(0)
```

## Contour Detection
---
