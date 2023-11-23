import cv2

img1_path = "data/s0.png"
img2_path = "data/s1.png"

img = cv2.imread(img2_path,0)
blurred = cv2.GaussianBlur(img,(15,15),0)
gaussImg = cv2.Canny(blurred, 20, 70)
cv2.imshow("Img",gaussImg)
cv2.waitKey(0)