import cv2
import numpy as np
from pyzbar.pyzbar import decode

def dist(angle,scale):
    rows, cols = img_base.shape[:2]
    center = (cols / 2, rows / 2)
    M = cv2.getRotationMatrix2D(center, angle, scale)
    cos = np.abs(M[0, 0])
    sin = np.abs(M[0, 1])
    new_cols = int((rows * sin) + (cols * cos))
    new_rows = int((rows * cos) + (cols * sin))
    M[0, 2] += (new_cols / 2) - center[0]
    M[1, 2] += (new_rows / 2) - center[1]
    img = cv2.warpAffine(img_base, M, (new_cols, new_rows))
    return img

def check(transformation, img):
    decoded = decode(img)
    try:
        assert len(decoded) != 0, "decoded is empty"
        found_left = False
        for value in decoded:
            if "left" in value.data.decode("utf-8").lower():
                found_left = True
                break
        assert found_left, "the word is not in the list"
        cv2.imshow(transformation, img)  
        cv2.waitKey(1)  
    except:
        print(transformation)
        cv2.imshow(transformation, img)  
        cv2.waitKey(1)  

img_base = cv2.imread("/home/francesca/Scrivania/MobileRobots_Project/bot/src/camera/qr_images/Left.png")

# BLUR
img = cv2.GaussianBlur(img_base, (5, 5), 10)  
check("blur",img)
img = cv2.GaussianBlur(img_base, (7, 7), 50)  
check("blur",img)
img = cv2.GaussianBlur(img_base, (11, 11), 80)  
check("blur",img)
img = cv2.GaussianBlur(img_base, (13,13), 100)  
check("blur",img)

# ROTATE
img = cv2.rotate(img_base, cv2.ROTATE_90_CLOCKWISE)  
check("rotate",img)
img = cv2.rotate(img_base, cv2.ROTATE_90_COUNTERCLOCKWISE)  
check("rotate",img)
img = cv2.rotate(img_base, cv2.ROTATE_180)  
check("rotate",img)

# BRIGHT
img = cv2.addWeighted(img_base, 1.5, np.zeros(img_base.shape, img_base.dtype), 0, 0) 
check("brightness",img)

# CONTRAST
img = cv2.convertScaleAbs(img_base, alpha=0.5)
check("contrast",img)
img = cv2.convertScaleAbs(img_base, alpha=0.1)
check("contrast",img)

# DIST
img = dist(30,1.0)
check("dist",img)
img = dist(45,1.0)
check("dist",img)
img = dist(75,1.0)
check("dist",img)
img = dist(100,1.0)
check("dist",img)
img = dist(135,1.0)
check("dist",img)

# RESIZE
img = cv2.resize(img_base, (int(img.shape[1]/2), int(img.shape[0]/2)))
check("resize",img)
img = cv2.resize(img_base, (int(img.shape[1]/3), int(img.shape[0]/3)))
check("resize",img)
img = cv2.resize(img_base, (int(img.shape[1]/4), int(img.shape[0]/4)))
check("resize",img)