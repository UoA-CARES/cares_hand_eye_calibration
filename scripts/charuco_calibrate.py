#!/usr/bin/env python

import numpy as np
import cv2
import cv2.aruco as aruco
from glob import glob
import pickle
import copy




def get_images(path, gray=False):
    """
    Given a path, it reads all images. This uses glob to 
    grab file names and excepts wild cards *
    Ex. getImages('./images/*.jpg')
    """
    imgs = []
    files = glob(path)
    files.sort()  # put in order

    print("Found {} images at {}".format(len(tuple(files)), path))
    # print('-'*40)

    for i, f in enumerate(files):
        img = cv2.imread(f)
        imgs.append(img)
    return imgs, files
#ROOT_PATH = "/home/anyone/scans/01-21/13-45-36/"
# ROOT_PATH = "/home/anyone/scans/01-25/14-49-33/"



class Calibrate(object):
    """Calibrate an individual camera"""
    def __init__(self):
        self.dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        x = 6  # horizontal
        y = 9  # vertical
        sqr = 0.039  # solid black squares
        mrk = 0.020 # markers, must be smaller than squares
        self.board = aruco.CharucoBoard_create(
            y,x,
            sqr,
            mrk,
            self.dictionary)
        board_img = self.board.draw((3000, 2000))
        cv2.imshow("sampleboard",board_img)
        cv2.waitKey(0)
        cv2.imwrite("board.png",board_img)
        print("saved")
        
    def calculateReprojectionError(self, imgpoints, objpoints, rvecs, tvecs, mtx, dist):
        """
        imgpts: features found in image, (num_imgs, 2)
        objpts: calibration known features in 3d, (num_imgs, 3)
        """
        imgpoints = [c.reshape(-1,2) for c in imgpoints]
        mean_error = 0
        for i in range(len(objpoints)):
#             print('img',imgpoints[i].shape)
#             print('obj', objpoints[i].shape)
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            imgpoints2 = imgpoints2.reshape(-1,2)
#             print('img2', imgpoints2.shape)
            
            # if not all markers were found, then the norm below will fail
            if len(imgpoints[i]) != len(imgpoints2):
                continue
                
            error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
            mean_error += error
        print( "total error: {}".format(mean_error/len(objpoints)) )
    

    def filterobjpoint(self, ids,objpoints):
        for i in reversed(range(40)):
            if i in ids:
                pass
            else:
                #print("deleting ",i)
                objpoints = np.delete(objpoints,i,0)
        return objpoints#

    def detectBoard(self,im):
        gray = im.copy()
                
        corners, ids, rejectedImgPts = aruco.detectMarkers(gray, self.dictionary)            
        # if ids were found, then
        if ids is not None and len(ids) > 0:
            ret, chcorners, chids = aruco.interpolateCornersCharuco(
                corners, ids, gray, self.board)
            #calcorners.append(chcorners)
            #print(chcorners)
        return ids, chcorners, chids, corners


    def detect_ID_POS(self,image, id):
        ids, chcorners, chids, corners = detectBoard(image)
        counter = [0]*40
        for chid in chids:
            for i in chid:
                counter[i]+=1
        common_id = counter.index(40)
        print(common_id, "is common id in all")



    def calibrate(self, imgs, removeIndex,K=None):
        calcorners = []  # 2d points in image
        calids = []  # ids found in image
        h,w = imgs[0].shape[:2]
        
        # so we know a little bit about the camera, gso
        # start off the algorithm with a simple guess
        f = max(h,w)  # focal length is a function of image size in pixels
        K = np.array([
            [float(f),0.0,w//2.0],
            [0.0,float(f),float(h)//2.0],
            [0.0,0.0,1.0]
        ])

        for i, im in enumerate(imgs):
            # make grayscale if it is not already
            #if len(im.shape) > 2:
            #    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            #else:
            #gray = im.copy()
                
            #corners, ids, rejectedImgPts = aruco.detectMarkers(gray, self.dictionary)
#            print("detectMarkers found {} corners {} ids".format(
#                  len(corners), len(ids)))
            
            # if ids were found, then
            ids, chcorners, chids, corners = self.detectBoard(im)
            if ids is not None and len(ids) > 0:
                #ret, chcorners, chids = aruco.interpolateCornersCharuco(
                #    corners, ids, gray, self.board)
                calcorners.append(chcorners)
                #print(chcorners)
                if chcorners is None or chcorners.shape[0] < 40:
                    removeIndex.add(i)
                self.detect_center(im)
                calids.append(chids)
                im2 = im.copy()
                aruco.drawDetectedCornersCharuco(im2, chcorners, chids)
                aruco.drawDetectedMarkers(im2, corners, ids=ids, borderColor=(100, 0, 240))
                cv2.namedWindow('board', cv2.WINDOW_NORMAL)
                cv2.imshow("board",im2)
                cv2.waitKey(10)
        flags = 0
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS  # make an inital guess at cameraMatrix (K)
#         flags |= cv2.CALIB_FIX_PRINCIPAL_POINT  # value? makes it worse
        #
        a = self.board.chessboardCorners.reshape((40,1,3))
        idx = 0
        for i in range(0,5):
            for j in reversed(range(0,8)):
                a[idx][0][0] = j*0.039
                a[idx][0][1] = i*0.039
                idx+=1
        #print(a)
        objpts = [a]*len(calcorners)
        b = self.board.chessboardCorners.reshape((40,1,3))
        objpts_orig = [b]*len(calcorners)
        for i in reversed(range(len(imgs))):
            if calids[i] is None:
                np.delete(calids,i,0)
                np.delete(objpts,i,0)
                continue
            objpts[i] = self.filterobjpoint(calids[i],objpts[i])

#         imgpts = [c.reshape(-1,2) for c in calcorners]
        imgpts =   calcorners                                              

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
        rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpts, imgpts,(4000,3000), None, None, criteria=criteria)
        #rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCameraCharuco(
        #    charucoCorners=calcorners,
        #    charucoIds=calids,
        #    board= self.board,
        #    imageSize = (w,h),
        #    cameraMatrix = K,
        #    distCoeffs=None,
        #    flags=flags)
        cam_params = {
            'marker_type': 'aruco',
            'cameraMatrix': cameraMatrix,
            'distCoeffs': distCoeffs,
            'image_size': imgs[0].shape[:2],
#             'marker_size': (x,y),
#             'marker_scale:': sqr
            'rms': rms
        }
#         objpts = [self.board.chessboardCorners.copy() for c in calcorners]
        h, w = self.board.chessboardCorners.shape


        #objpts = [self.board.chessboardCorners.reshape((h,1,3)) for c in calcorners]
        
        
        #print('obj', len(objpts))
        #print('imgpts', len(imgpts))
        
        self.calculateReprojectionError(imgpts, objpts, rvecs, tvecs, cameraMatrix, distCoeffs)
        
        return (rms, cameraMatrix, distCoeffs, rvecs, tvecs, objpts, imgpts, calids)


    
def filterpoints(l_cid, r_cid, l_point, r_point, obj_point):
    l_point_lst = l_point.tolist()
    r_point_lst = r_point.tolist()
    obj_point_lst = obj_point.tolist()
    #print("l_id",l_cid)
    #print("r_id",r_cid)
    #print(len(obj_point_lst))
    #print(obj_point_lst)
    for i in reversed(range(40)):
        if [i] in l_cid and [i] in r_cid:
            continue
        else:
            if [i] in l_cid:
                print("removing from left",i)
                idx = l_cid.tolist().index([i])
                #print("pos",idx)
                del l_point_lst[idx]
            if [i] in r_cid:
                print("removing ",i)
                idx = r_cid.tolist().index([i])
                #print("pos",idx)
                del r_point_lst[idx]
            #del obj_point_lst[i]
    l_point = np.array(l_point_lst,dtype="float32")
    r_point = np.array(r_point_lst,dtype="float32")
    obj_point = np.array(obj_point_lst,dtype="float32")
    return l_point, r_point, obj_point

def save_calibration(K1,K2,imageSize,dist1, dist2, R, T, P1, R1, P2, R2, Q, imageSet ,root_path):
    calib = {}    
    #cam1    
    camera1_calibration ={}
    camera1_calibration["image_size"]=imageSize
    camera1_calibration["K"] = K1
    camera1_calibration["dist"] = dist1
    #camera 2
    camera2_calibration ={}
    camera2_calibration["image_size"]=imageSize
    camera2_calibration["K"] = K2
    camera2_calibration["dist"] = dist2
    #extrinsics
    camera_extrinsics = {}
    camera_extrinsics["R"] = R
    camera_extrinsics["T"] = T
    camera_extrinsics["parent"] ="cam1"
    camera_extrinsics["Q"]="Q"
    #total
    calib={}     
    calib["cameras"] = {"cam1":camera1_calibration, "cam2":camera2_calibration}
    calib["extrinsics"]  = {"cam2":camera_extrinsics}
    calib["stereo_pairs"] = {"stereo1":["cam1", "cam2"]}
    calib["image_sets"] = {"rgb":imageSet}
    import json
    file_name = root_path+"calibration.json"
    file_name2 = root_path+"calibration2.json"
    with open(file_name,"w") as fp:
        class NumpyEncoder(json.JSONEncoder):
            def default(self, obj):
                if isinstance(obj, np.ndarray):
                    return obj.tolist()
                return json.JSONEncoder.default(self, obj)
        json.dump(calib, fp, cls=NumpyEncoder)
        print("saved ",file_name)
    fs = cv2.FileStorage(file_name2,cv2.FileStorage_WRITE)
    fs.write("imageSize",imageSize)
    fs.write("K1",K1)
    fs.write("dist1",dist1)
    fs.write("K2",K1)
    fs.write("dist2",dist2)
    fs.write("R",R)
    fs.write("T",T)
    fs.write("P1",P1)
    fs.write("P2",P2)
    fs.write("R1",R1)
    fs.write("R2",R2)
    fs.write("Q",Q)
    #fs.write("imageSet",imageSet)
    fs.release()


def readFromFiles(root_path):
    ROOT_PATH = root_path
    imgsL, cam1_files = get_images(ROOT_PATH+"*left_rgb.png", gray=True)
    imgsR, cam2_files = get_images(ROOT_PATH+"*right_rgb.png", gray=True)
    M1, M2, d1, d2, w, h, R, T, P1, P2, R1, R2,  Q = charuco_main(imgsL,imgsR)
    imageSet = [cam1_files,cam2_files]
    #save_calibration(K1,K2,imageSize,dist1, dist2, R, T, imageSet ):
    save_calibration(M1,M2,(w,h),d1,d2,R,np.transpose(T)[0],P1, R1, P2, R2, Q,imageSet,ROOT_PATH)

def charuco_main(imgsL, imgsR):
    cal = Calibrate()
    removeIndex = set()
    rms1, M1, d1, r1, t1, objpoints, imgpoints_l, l_calids = cal.calibrate(imgsL,removeIndex)
    #print(len(objpoints), objpoints[0].shape, len(imgpoints_l), imgpoints_l[0].shape)
    print('RMS:', rms1, 'px')
    print('Camera Matrix:', M1)
    print('Dist Coeffs:', d1)

    #img_undist = cv2.undistort(imgsL[i],M1,d1,None)

    rms2, M2, d2, r2, t2, objpoints, imgpoints_r, r_calids = cal.calibrate(imgsR,removeIndex)

    print('RMS:', rms2, 'px')
    print('Camera Matrix:', M2)
    print('Dist Coeffs:', d2)

    #img_undist = cv2.undistort(imgsR[i],M2,d2,None)




    ##EXTRINSICS
    #print('obj pts', objpoints)
    #print('imgpoints left', imgpoints_l[0])
    #print('imgpoints right', imgpoints_r[0])

    flags = 0
    # flags |= cv2.CALIB_FIX_INTRINSIC
    # flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    flags |= cv2.CALIB_USE_INTRINSIC_GUESS
    flags |= cv2.CALIB_FIX_FOCAL_LENGTH
    # flags |= cv2.CALIB_FIX_ASPECT_RATIO
    # flags |= cv2.CALIB_ZERO_TANGENT_DIST
    # flags |= cv2.CALIB_RATIONAL_MODEL
    # flags |= cv2.CALIB_SAME_FOCAL_LENGTH
    # flags |= cv2.CALIB_FIX_K3
    # flags |= cv2.CALIB_FIX_K4
    # flags |= cv2.CALIB_FIX_K5

    stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER +
                            cv2.TERM_CRITERIA_EPS, 100, 1e-5)

    h, w = imgsL[0].shape[:2]

    print(removeIndex)
    remove_list = list(removeIndex)
    remove_list.sort()
    #for i in reversed(remove_list):
    #    del imgpoints_l[i]
    #    del imgpoints_r[i]
    #    del objpoints[i]
    for i in range(len(imgpoints_l)):
        lp, rp, op = filterpoints(l_calids[i], r_calids[i], imgpoints_l[i], imgpoints_r[i], objpoints[i])
        imgpoints_l[i] = lp
        imgpoints_r[i] = rp
        objpoints[i] = op
    #STEREOOOOOOOOOOOOOOOO
    ret, M1_rec, d1_rec, M2_rec, d2_rec, R, T, E, F = cv2.stereoCalibrate(
        objpoints,
        imgpoints_l,
        imgpoints_r,
        M1, d1,
        M2, d2,
        (w,h),
        criteria=stereocalib_criteria,
        flags=flags)

    ret, M1_rec, d1_rec, M2_rec, d2_rec, R, T, E, F, perViewError= cv2.stereoCalibrateExtended(
        objpoints,
        imgpoints_l,
        imgpoints_r,
        M1, d1,
        M2, d2,
        (w,h),
        R, T,
        criteria=stereocalib_criteria,
        flags=flags)

    print('-'*50)
    #print('Image: {}x{}'.format(*imgs_l[0].shape[:2]))
    # print('{}: {}'.format(marker_type, marker_size))
    print('Intrinsic Camera Parameters')
    print('-'*50)
    print(' [Camera 1]')
    # print('  cameraMatrix_1', M1)
    print('  f(x,y): {:.1f} {:.1f} px'.format(M1[0,0], M1[1,1]))
    print('  principlePoint(x,y): {:.1f} {:.1f} px'.format(M1[0,2], M1[1,2]))
    print('  distCoeffs', d1[0])
    print(' [Camera 2]')
    # print('  cameraMatrix_2', M2)
    print('  f(x,y): {:.1f} {:.1f} px'.format(M2[0,0], M2[1,1]))
    print('  principlePoint(x,y): {:.1f} {:.1f} px'.format(M2[0,2], M2[1,2]))
    print('  distCoeffs', d2[0])
    print('-'*50)
    print('Extrinsic Camera Parameters')
    print('-'*50)
    print('  R', R)
    print('  T[meter]', T)
    print('  E', E)
    print('  F', F)
    print('  error', ret)
    print('  stereo_rms', perViewError)


    print('Object Points in 3D [meters]')
    print('images:',len(objpoints))
    print('features found per image:',objpoints[0].shape)
    #print('point:',objpoints[0][0])


    print('Image Points in 2D [pixels]')
    print('images:',len(imgpoints_l))
    print('features found per image:',imgpoints_l[0].shape)
    #print('point:',imgpoints_l[0][0])


    print('Image Points in 2D [pixels]')
    #print('images:',len(imgpoints_r))
    #print('features found per image:',imgpoints_r[0].shape)
    #print('point:',imgpoints_r[0][0])

    #imgpoints_l[0].shape
    recL, recR, P1, P2, R1, R2, Q = rectify_remap(imgsL, imgsR, M1, d1, M2, d2, w, h, R, T)
    #objpoints[0].shape
    for i in range(len(recL)):
        im_h = cv2.hconcat([recL[i], recR[i]])
        for h in range(0, im_h.shape[0],40):
            cv2.line(im_h, (0, h), (im_h.shape[1]*2-1, h), (0, 255, 0), thickness=4)
        cv2.imshow("rectified",im_h)
        #print(im_h.shape)
        cv2.waitKey(100)
    return M1, M2, d1, d2, w, h, R, T, P1, P2, R1, R2,  Q

def rectify_remap(imgsL, imgsR, M1, d1, M2, d2, w, h, R, T):
    cv2.namedWindow('rectified', cv2.WINDOW_NORMAL)
    R1, R2, P1, P2, Q, roi_left, roi_right = cv2.stereoRectify(M1, d1, M2, d2, (w,h), R, T, flags=cv2.CALIB_ZERO_DISPARITY, alpha=1.0)
    leftMapX, leftMapY = cv2.initUndistortRectifyMap(M1, d1, R1, P1, (w,h), cv2.CV_32FC1)
    rightMapX, rightMapY = cv2.initUndistortRectifyMap(M2, d2, R2, P2, (w,h), cv2.CV_32FC1)
    imgsL_rec = []
    imgsR_rec = []
    for i in range(len(imgsL)):
        #print(cv2.remap)
        left_rectified = cv2.remap(imgsL[i], leftMapX, leftMapY, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)    
        right_rectified = cv2.remap(imgsR[i], rightMapX, rightMapY, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        imgsL_rec.append(left_rectified)
        imgsR_rec.append(right_rectified) 
    return imgsL_rec, imgsR_rec, P1, P2, R1, R2, Q

if __name__=="__main__":
    readFromFiles("/home/anyone/scans/2021-03-22-21-45-12/")