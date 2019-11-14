#!/usr/bin/env python
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F 
from torchvision import datasets, transforms, models
from scipy.misc import imread, imresize
from lib.nn import async_copy_to
import cv2
from models import ModelBuilder, SegmentationModule
import matplotlib.pyplot as plt
import timeit
from lib.utils import as_numpy
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from utils import colorEncode
from scipy.io import loadmat
img = None
colors = loadmat('/home/crrl/crrl_ws/src/visguide/scripts/semantic-segmentation-pytorch/data/color150.mat')['colors']
bridge = CvBridge()

#i = 0
def callback(data):
		#start = timeit.default_timer()
		#global i
		#i+=1
		img1 = bridge.imgmsg_to_cv2(data, "bgr8")
		seg = np.zeros((img1.shape[0], img1.shape[1], 1)).astype(np.uint8)
		seg_size = (img1.shape[0], img1.shape[1])

		img = img1.astype(np.float32)
		img = img.transpose((2,0,1))
		img = img_transform(torch.from_numpy(img))
		img = torch.unsqueeze(img, 0)
		feed_dict = async_copy_to({"img_data": img.half()}, 0)
		pred = segmentation_module(feed_dict, segSize=seg_size)
		
		pred,ind = torch.max(pred, dim=1)
		ind = as_numpy((ind.squeeze()).cpu())

		seg[:,:,0] = ind
		
		im = bridge.cv2_to_imgmsg(seg, "mono8")
		#print(np.array_equal(seg, np.int8(seg)))
		#print(np.array_equal(seg, np.int32(seg)))
		seg[seg != 1] = 0
		# cv2.imshow('im', np.int32(seg))
		# cv2.waitKey(1)
		
		im_label = bridge.cv2_to_imgmsg(np.int32(seg), "32SC1")
		
		im.header = data.header
		im_label.header = data.header
		pub.publish(im)
		pub_label.publish(im_label)
		#stop = timeit.default_timer()
		#print(stop-start)


img_transform = transforms.Normalize(mean=[102.9801, 115.9465, 122.7717], std=[1., 1., 1.])
device = torch.cuda.set_device(0)


builder = ModelBuilder()
net_encoder = builder.build_encoder(
        arch='resnet18dilated',
        fc_dim=512,
		weights="/home/crrl/crrl_ws/src/visguide/scripts/semantic-segmentation-pytorch/ckpt/baseline-resnet18dilated-c1_deepsup/encoder_epoch_2.pth")

net_decoder = builder.build_decoder(
        arch='c1_deepsup',
        fc_dim=512,
        num_class=12,
        weights="/home/crrl/crrl_ws/src/visguide/scripts/semantic-segmentation-pytorch/ckpt/baseline-resnet18dilated-c1_deepsup/decoder_epoch_2.pth",
		use_softmax=True)

crit = nn.NLLLoss(ignore_index=-1)
segmentation_module = SegmentationModule(net_encoder, net_decoder, crit)
segmentation_module.half()
segmentation_module.cuda()
segmentation_module.eval()

rospy.init_node('Semantic_segmentation', anonymous=True)
rospy.Subscriber('/visguide/zed_node/rgb/image_rect_color', Image, callback)
pub=rospy.Publisher('/visguide/zed_node/seg/image_rect_color', Image, queue_size=30)
pub_label = rospy.Publisher('/visguide/zed_node/label/image_rect_color', Image, queue_size=30)
rospy.spin()
#callback(0)




