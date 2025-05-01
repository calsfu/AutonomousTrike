import cv2
import numpy as np
import matplotlib.pyplot as plt

class Emergency_Brake:
    def __init__(self, noise_thresh, kblur, safe_cm, boxes, box_ring, show):
        # self.noise_thresh = noise_thresh
        # self.kblur = kblur
        self.safe_cm = safe_cm
        # self.boxes = boxes
        # self.box_ring = box_ring
        # self.show = show
        
        self.noise_thresh = 1000
        self.kblur = 19
        # self.safe_cm = 3000
        self.boxes = 6
        self.box_ring = 1 # can't be > boxes / 2

    def get_brake(self, npimg_in):
        npimg = np.copy(npimg_in).astype(np.uint16)
        datatype = npimg.dtype
        dmax = np.iinfo(datatype).max
        # image processing
        edge = cv2.Sobel(src=npimg, ddepth=cv2.CV_16UC1, dx=1, dy=1, ksize=7)
        # edge = np.asarray(edge_out, dtype=float)
        
        # plt.figure()
        # plt.subplot(1,2,1)
        # plt.imshow(npimg)
        # plt.subplot(1,2,2)
        # plt.imshow(edge)
        # plt.savefig("figs/eb.jpg", dpi=100)
        # plt.close()

        ksize = (self.kblur, self.kblur) 
        blur = cv2.blur(edge, ksize)  
        # print(blur_out)
        # blur = np.asarray(blur_out)
        # print(blur_out.dtype)
        # blur = cv2.GaussianBlur(edge, (19, 19), 0)
        # blur = cv2.medianBlur(edge, 5)
        cleaned_img_zeros = np.where(blur < self.noise_thresh, npimg, 0)
        # cleaned_img_zeros = np.where(blur_out < 1000, npimg, 0)
        cleaned_img_np = np.where(cleaned_img_zeros == 0, np.nan, npimg)
        # except:
        #     print("blur broke")
        #     return
        # finding nearest object
        x, y = cleaned_img_np.shape
        x, y = int(x / self.boxes), int(y / self.boxes)
        box_avgs = np.empty([self.boxes, self.boxes])
        for i in range(self.boxes):
            for j in range(self.boxes):
                box = cleaned_img_np[x * i:x * (i + 1),y * j:y * (j + 1)]
                box_avgs[i,j] = np.nanmean(box)
        box_avgs = box_avgs.astype(datatype)
        box_avgs = box_avgs[self.box_ring:-self.box_ring, self.box_ring:-self.box_ring]
        box_avgs = np.where(box_avgs == 0, dmax, box_avgs)
        # if self.show:
        #     print(box_avgs)
        #     plt.figure()
        #     plt.subplot(1,3,1)
        #     plt.imshow(npimg)
        #     plt.subplot(1,3,2)
        #     plt.imshow(edge)
        #     plt.subplot(1,3,3)
        #     plt.imshow(cleaned_img_np)
        #     plt.savefig("figs/eb.jpg", dpi=100)
        #     plt.close()
        # publishing value
        if(np.min(box_avgs) < self.safe_cm):
            return 1
        else:
            return 0
