import cv2

img1_path = "data/1.png"
img2_path = "data/s1.png"
img3_path = "data/s2.jpg"
img4_path = "data/s3.jpg"

img = cv2.imread(img4_path,0)
blurred = cv2.GaussianBlur(img,(15,15),0)
gaussImg = cv2.Canny(blurred, 20, 70)
cv2.imshow("Img",gaussImg)
cv2.waitKey(0)