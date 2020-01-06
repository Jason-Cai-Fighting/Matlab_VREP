# -*- coding: utf-8 -*-
"""
Created on Mon Jan  6 09:58:20 2020

@author: Administrator
"""
from skimage import io
import os

img_path='../../img/' #图像路径
img_list=os.listdir(img_path)
img_num=len(os.listdir(img_path))   #计算目录下有多少图像
os.remove(img_path+img_list[0])     #移除第一个无效文件


for i in range(1,img_num):
    img_name = img_path + img_list[i]
    im = io.imread(img_name)
    im = im[:,:,:3]
    io.imsave(img_name, im)