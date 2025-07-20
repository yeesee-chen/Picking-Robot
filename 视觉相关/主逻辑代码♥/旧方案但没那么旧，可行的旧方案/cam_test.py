#coding:utf-8
import cv2
cap = cv2.VideoCapture(1)#图像源，若为外置摄像头则将0改为1
flag = cap.isOpened()
path = 'D:' #图片储存路径，中文路径有概率存储失败
index = 1 #第一位序号
while(flag):
    ret, frame = cap.read()
    cv2.imshow("Capture_Paizhao",frame)
    k = cv2.waitKey(1) & 0xFF
    if k == ord('s'):     #按下s键，进入下面的保存图片操作
        img_name = '%s/%d.jpg' % (path, index)
        cv2.imwrite(img_name,frame)
        print(cap.get(3))
        print(cap.get(4))
        print("save" + str(index) + ".jpg successfuly!")
        print("-------------------------")
        index += 1
    elif k == ord('q'):     #按下q键，程序退出
        break
cap.release()
##cv2.destroyAllWindows()