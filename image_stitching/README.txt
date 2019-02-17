CS281B Homework Assignment #3: Panorama Stitching
Author: Yitian Shao
=========================================================================
Program running instruction:
1. Please ensure the folder ('GrandCanyon1') containing source images is 
   in the same path as MATLAB files. It contains 10 randomly ordered images.

2. Please ensure the VLFeat library ('vlfeat-0.9.20') is included.

3. Then please run the script 'HW3_Main_Yitian.m'.

4. Please include image folders in the same path and modify the parameter
   'Data_path' in the main script 'HW3_Main_Yitian.m', when testing other 
   set of images.
=========================================================================
This code can run in MATLAB R2015b or later (Untested in previous vision)

Due to high computation cost of the program. Only stitching the included 
set of images are guranteed. Other images, especially those requiring 360
cylindrical stitching, may cause some problem. 
=========================================================================
Program description:
1. This program can stitch images with same name in designated path to
   produce panaroma image. 
2. This program can take images in RANDOM SEQUENCES.
3. The stitched panorama will be interpolated to reduce unwanted
   artifacts caused by stitching.
==========================================================================
The feature extraction part of this program is implemented using the 
VLFeat open source library - http://www.vlfeat.org/.
Functions used: 'vl_sift' and 'vl_ubcmatch'. 
Everything else is self-written.

VLFeat setup instruction - http://www.vlfeat.org/install-matlab.html
=========================================================================
MATLAB path looks like this:
-------------------------------
Folder
-------------------------------
'GrandCanyon1'
'vlfeat-0.9.20'

-------------------------------
Function
-------------------------------
amend.m
estimateHomography.m
featureExtraction.m
plotFeature.m
ransac.m
slctFeature.m
stitchImage2.m
stitchSeq.m

-------------------------------
Script
-------------------------------
HW3_Main_Yitian.m

-------------------------------
Text Document
-------------------------------
README.txt
-------------------------------
=========================================================================
My contact: yitianshao@gmail.com