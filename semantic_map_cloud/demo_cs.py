#!/usr/bin/env python

# --------------------------------------------------------
# Faster R-CNN
# Copyright (c) 2015 Microsoft
# Licensed under The MIT License [see LICENSE for details]
# Written by Ross Girshick
# --------------------------------------------------------

"""
Demo script showing detections in sample images.

See README.md for installation instructions before running.
"""

import _init_paths
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms
from utils.timer import Timer
#from PIL import Image, ImageGrab
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
import caffe, os, sys, cv2
import argparse
import Image
import rospy
import math
import linecache
#import thread
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from math import radians, copysign, sqrt, pow, pi
#from multiprocessing import Process



CLASSES = ('__background__',
           'aeroplane', 'bicycle', 'bird', 'boat',
           'bottle', 'bus', 'car', 'cat', 'chair',
           'cow', 'diningtable', 'dog', 'horse',
           'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor')

NETS = {'vgg16': ('VGG16',
                  'VGG16_faster_rcnn_final.caffemodel'),
        'zf': ('ZF',
                  'ZF_faster_rcnn_final.caffemodel')}
'''
def callback1(data):#right-down
    global obx1
    obx1 = data.data
    #print('111111')
def callback2(data):
    global oby1
    oby1 = data.data
def callback3(data):#right-up
    global obx2
    obx2 = data.data
def callback4(data):
    global oby2
    oby2 = data.data
def callback5(data):#left-down
    global obx3
    obx3 = data.data
def callback6(data):
    global oby3
    oby3 = data.data
def callback7(data):#left-up
    global obx4
    obx4 = data.data
def callback8(data):
    global oby4
    oby4 = data.data
    #print('444444')
'''



def vis_detections(im, class_name, dets, k, thresh=0.5):
    """Draw detected bounding boxes."""
    inds = np.where(dets[:, -1] >= thresh)[0]
    if len(inds) == 0:
        return

    im = im[:, :, (2, 1, 0)]
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.imshow(im, aspect='equal')
    f = open("/home/lyy/py-faster-rcnn/tools/result_test.txt", "a")

    for i in inds:
        bbox = dets[i, :4]
        score = dets[i, -1]
        
        print >> f, ("[" + bytes(bbox[0]) + ", " + bytes(bbox[1]) + ", " + bytes(bbox[2]) + ", " + bytes(bbox[3]) + "]") #left-up point(x0,y0) right-down point(x1,y1)
        ave_x = 0.5*(float(bytes(bbox[0])) + float(bytes(bbox[2])))
        ave_y = 0.5*(float(bytes(bbox[1])) + float(bytes(bbox[3])))
        #print >> f, (ave_x)
        #print >> f, (ave_y)
        if(ave_x <= 320 and ave_y <=240):
            flag = 4 #left-up
            #print >> f, (flag)
            ox4 = linecache.getline('/home/lyy/semantic_map/out.txt',linecount-3)
            oy4 = linecache.getline('/home/lyy/semantic_map/out.txt',linecount-2)
            print >> f,(ox4)
            print >> f,(oy4)
            #print >> f, (obx4)
            #print >> f, (oby4)
        if(ave_x <= 320 and ave_y <=480 and ave_y > 240):
            flag = 3 #left-down
            #print >> f, (flag)
            ox3 = linecache.getline('/home/lyy/semantic_map/out.txt',linecount-5)
            oy3 = linecache.getline('/home/lyy/semantic_map/out.txt',linecount-4)
            print >> f,(ox3)
            print >> f,(oy3)
            #print >> f, (obx3)
            #print >> f, (oby3)
        if(ave_x <= 640 and ave_x > 320 and ave_y <=240):
            flag = 2 #right-up
            #print >> f, (flag)
            ox2 = linecache.getline('/home/lyy/semantic_map/out.txt',linecount-7)
            oy2 = linecache.getline('/home/lyy/semantic_map/out.txt',linecount-6)
            print >> f,(ox2)
            print >> f,(oy2)
            #print >> f, (obx2)
            #print >> f, (oby2)
        if(ave_x <= 640 and ave_x > 320 and ave_y <=480 and ave_y > 240):
            flag = 1 #right-down
            #print >> f, (flag)
            ox1 = linecache.getline('/home/lyy/semantic_map/out.txt',linecount-9)
            oy1 = linecache.getline('/home/lyy/semantic_map/out.txt',linecount-8)
            print >> f,(ox1)
            print >> f,(oy1)
            #print >> f, (obx1)
            #print >> f, (oby1)

        print >> f, (score)
        

        if score >= 0.7:
            ax.add_patch(
            plt.Rectangle((bbox[0], bbox[1]),
                          bbox[2] - bbox[0],
                          bbox[3] - bbox[1], fill=False,
                          edgecolor='blue', linewidth=3.5)
            )
            print >>f, (class_name + "\n")
            ax.text(bbox[0], bbox[1] - 2,
                '{:s} {:.3f}'.format(class_name, score),
                bbox=dict(facecolor='blue', alpha=0.5),
                fontsize=14, color='white')

        if score < 0.7:
            ax.add_patch(
            plt.Rectangle((bbox[0], bbox[1]),
                          bbox[2] - bbox[0],
                          bbox[3] - bbox[1], fill=False,
                          edgecolor='red', linewidth=3.5)
            )
            plt.axis('off')
            plt.tight_layout()
            k = bytes(k)
            plt.savefig("/home/lyy/py-faster-rcnn/tools/test_pic/"+k+".jpg")
            img = Image.open("/home/lyy/py-faster-rcnn/tools/test_pic/"+k+".jpg")
            region = (int(bbox[0]*2.24)+64, int(bbox[1]*2.26)+180, int(bbox[2]*2.24)+64, int(bbox[3]*2.26)+180) #for 500*375 pixels
            #region = (int(bbox[0]*1.75)+64, int(bbox[1]*1.77)+180, int(bbox[2]*1.75)+64, int(bbox[3]*1.77)+180) #for 640*480(original) to 1200*1200(detected)
            cropImg = img.crop(region)
            i = bytes(i)
            cropImg.save("/home/lyy/py-faster-rcnn/tools/test_pic/"+k+"_"+i+"_crop.jpg")
            f1 = open("/home/lyy/py-faster-rcnn/tools/cs_result.txt","w+")
            os.system("python /home/lyy/py-faster-rcnn/tools/cs.py " + "/home/lyy/py-faster-rcnn/tools/test_pic/"+k+"_"+i+"_crop.jpg" + " >> " + "/home/lyy/py-faster-rcnn/tools/cs_result.txt")
            cs_res = open("/home/lyy/py-faster-rcnn/tools/cs_result.txt").read()
            f1.close()
            print >>f, (cs_res)
            ax.text(bbox[0], bbox[1] - 2,
                '{:s}'.format(cs_res),
                bbox=dict(facecolor='red', alpha=0.5),
                fontsize=14, color='white')



    ax.set_title(('{} detections with '
                  'p({} | box) >= {:.1f}').format(class_name, class_name,
                                                  thresh),
                  fontsize=14)
    plt.axis('off')
    plt.tight_layout()
    plt.draw()
    k = bytes(k)
    print >>f, ("The final pic: " + k + ".jpg")
    print >>f, ('~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n')
    f.close()
    print("ok")
    plt.savefig("/home/lyy/py-faster-rcnn/tools/test_pic/" + k + ".jpg")

def demo(net, image_name):
    """Detect object classes in an image using pre-computed object proposals."""
    
    file = open('/home/lyy/semantic_map/out.txt','r')
    global linecount
    linecount = len(file.readlines()) #read semantic_map/out.txt object_position

    # Load the demo image
    im_file = os.path.join(cfg.DATA_DIR, 'demo', image_name)
    im = cv2.imread(im_file)

    # Detect all object classes and regress object bounds
    timer = Timer()
    timer.tic()
    scores, boxes = im_detect(net, im)
    timer.toc()
    print ('Detection took {:.3f}s for '
           '{:d} object proposals').format(timer.total_time, boxes.shape[0])

    # Visualize detections for each class
    CONF_THRESH = 0.1
    NMS_THRESH = 0.5
    k=1
    for cls_ind, cls in enumerate(CLASSES[1:]):
        cls_ind += 1 # because we skipped background
        cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
        cls_scores = scores[:, cls_ind]
        dets = np.hstack((cls_boxes,
                          cls_scores[:, np.newaxis])).astype(np.float32)
        keep = nms(dets, NMS_THRESH)
        dets = dets[keep, :]
        vis_detections(im, cls, dets, k, thresh=CONF_THRESH)
        k = k+1
    
    linecache.clearcache() #clearcache to read the next image object position information
    
def parse_args():
    """Parse input arguments."""
    parser = argparse.ArgumentParser(description='Faster R-CNN demo')
    parser.add_argument('--gpu', dest='gpu_id', help='GPU device id to use [0]',
                        default=0, type=int)
    parser.add_argument('--cpu', dest='cpu_mode',
                        help='Use CPU mode (overrides --gpu)',
                        action='store_true')
    parser.add_argument('--net', dest='demo_net', help='Network to use [vgg16]',
                        choices=NETS.keys(), default='vgg16')

    args = parser.parse_args()

    return args

'''
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/object_x1", Float64, callback1)
    rospy.Subscriber("/object_y1", Float64, callback2)
    rospy.Subscriber("/object_x2", Float64, callback3)
    rospy.Subscriber("/object_y2", Float64, callback4)
    rospy.Subscriber("/object_x3", Float64, callback5)
    rospy.Subscriber("/object_y3", Float64, callback6)
    rospy.Subscriber("/object_x4", Float64, callback7)
    rospy.Subscriber("/object_y4", Float64, callback8)
    rospy.spin()
'''
#def my_fork():
#    listener()

if __name__ == '__main__':
    #p = Process(target = my_fork)
    #p.start()

    #thread.start_new_thread(listener,())

    cfg.TEST.HAS_RPN = True  # Use RPN for proposals

    args = parse_args()

    prototxt = os.path.join(cfg.MODELS_DIR, NETS[args.demo_net][0],
                            'faster_rcnn_alt_opt', 'faster_rcnn_test.pt')
    caffemodel = os.path.join(cfg.DATA_DIR, 'faster_rcnn_models',
                              NETS[args.demo_net][1])

    if not os.path.isfile(caffemodel):
        raise IOError(('{:s} not found.\nDid you run ./data/script/'
                       'fetch_faster_rcnn_models.sh?').format(caffemodel))

    if args.cpu_mode:
        caffe.set_mode_cpu()
    else:
        caffe.set_mode_gpu()
        caffe.set_device(args.gpu_id)
        cfg.GPU_ID = args.gpu_id
    net = caffe.Net(prototxt, caffemodel, caffe.TEST)

    print '\n\nLoaded network {:s}'.format(caffemodel)

    # Warmup on a dummy image
    im = 128 * np.ones((300, 500, 3), dtype=np.uint8)
    for i in xrange(2):
        _, _= im_detect(net, im)

    #im_names = ['000456.jpg', '000542.jpg', '001150.jpg',
                #'001763.jpg', '004545.jpg']
    im_names = ['2.jpg']
    for im_name in im_names:
        print '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
        print 'Demo for data/demo/{}'.format(im_name)
        demo(net, im_name)

    #plt.show()
