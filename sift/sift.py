import cv2
import numpy as np

# https://docs.opencv.org/4.x/d1/de0/tutorial_py_feature_homography.html https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_feature2d/py_matcher/py_matcher.html https://colab.research.google.com/github/YoniChechik/AI_is_Math/blob/master/c_08_features/sift.ipynb

class SIFT(object):

    def __init__(self):
        # Initiate SIFT detector
        self.sift = cv2.SIFT_create()
        self.draw_params = {"matchColor":(0,255,0), "singlePointColor": None, "flags": 2}
        self.robo_r = cv2.imread("/home/alicanto/AlicantoFS/sift/letters/roboR/r0.png")
        self.robo_n = cv2.imread("/home/alicanto/AlicantoFS/sift/letters/roboN/n0.png")
        self.robo_b = cv2.imread("/home/alicanto/AlicantoFS/sift/letters/black/b0.png")
        #self.robo_r = cv2.imread("./letters/roboR/r0.png")
        #self.robo_n = cv2.imread("./letters/roboN/n0.png")
    
    def set_query_img(self, img_query):
        self.img_query =  img_query
        self.kp_q, self.des_q = self.sift.detectAndCompute(self.img_query, None)
    
    def filter_grass(self, query_img):
        # retorna True si cree que hay solo pasto
        canny_query = cv2.Canny(query_img,100,200)
        ones = np.count_nonzero(canny_query)
        if ones > 10:
            return False
        else:
            return True
        
    
    def get_sift_matches(self, img_train, FLANN_INDEX_KDTREE = 1):
        # find the keypoints and descriptors with SIFT
        kp_t, des_t = self.sift.detectAndCompute(img_train, None)

        index_params, search_params =  {"algorithm":FLANN_INDEX_KDTREE, "trees": 5}, {"checks":50}
        
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(self.des_q, des_t, k=2)
        # store all the good matches as per Lowe's ratio test.
        temp = list(filter(lambda m: m[0].distance < 0.7*m[1].distance, matches))
        if len(temp) == 0:
            return None, None
        good = np.array(temp)[:,0]
        
        src_pts = np.float32([ self.kp_q[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp_t[m.trainIdx].pt for m in good ]).reshape(-1,1,2)

        if len(dst_pts) > 4:
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            matchesMask = mask.ravel() # only inliers
        
            return kp_t, good[np.where(matchesMask == 1)]
        return None, None
    

    def get_keypoints(self, kp_t, good, max_pts=10):
        if max_pts > int(len(good)):
            max_pts = len(good)
        
        return np.float32([ kp_t[m.trainIdx].pt for m in good[:max_pts]]).reshape(max_pts, 2)


    def draw_keypoints(self, img, key_pts):
        for pt in key_pts:
            pt_pix = (int(pt[0]), int(pt[1]))
            cv2.circle(img, pt_pix, 5, (0,0,255), -1, 5)
                        

    def draw_matches(self, img_train, kp_t, good):
        # draw matches between query image and the img_train
        return cv2.drawMatches(self.img_query, self.kp_q, img_train, kp_t, good, None, **self.draw_params)

if __name__ == '__main__':

    sift = SIFT()
    
    img_query = cv2.imread("letters/roboR/r0.png")  # imagen del dataset recortada
    img_train = cv2.imread("letters/roboR/r3.png")  # imagen a consultar
    #img_train = cv2.imread("letters/roboR/r1.png")

    sift.set_query_img(img_query)
    kp_t = None
    good = None

    kp_t, good = sift.get_sift_matches(img_train)
    if kp_t is not None and good is not None:
        key_pts = sift.get_keypoints(kp_t, good)
        if len(key_pts) > 0:
            print("Found match!")
            print(key_pts)
        
        img_train_with_kp = img_train.copy()
        sift.draw_keypoints(img_train_with_kp, key_pts)
        # show img_train_copy
        
        img_matches = sift.draw_matches(img_train, kp_t, good)
        # show img_matches
        cv2.imwrite("matches.png", img_matches)
    else:
        print("No match!")