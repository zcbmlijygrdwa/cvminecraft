# Marr-Hildreth Edge Detection - implementation in Matlab


Author: Zhenyu Yang

Email: zhenyuyang@ucsb.edu

The Marr– Hildreth calculation is a strategy for recognizing edges in images, that is, constant bends where there are stable and fast varieties in picture brilliance. The Marr– Hildreth edge identification technique is simple and works by convolving the picture with the Laplacian of the Gaussian capacity, or, as a quick estimation by the contrast of Gaussians. At that point, zero intersections are distinguished in the separated outcome to get the edges. The Laplacian-of-Gaussian picture administrator is now and then likewise alluded to as the Mexican cap wavelet because of its visual shape when flipped around. David Marr and Ellen C. Hildreth are two of the innovators.

In this project, Hildreth edge detection method was implemented with Matlab.

All functions and scripts were implemented from scratch. Matlab Computer Vision Toolbos is not required.

Either prog1(imagePath) or my_edge_detector(imagePath) can be the entry point to this project.

### References
* **http://www.songho.ca/dsp/convolution/convolution2d_example.html**
* **http://www.cs.ucsb.edu/~cs181b/hw/prog1.pdf**
* **https://blog.csdn.net/bettyshasha/article/details/51757185**
* **https://blog.csdn.net/jorg_zhao/article/details/52687448**
* **http://homepages.inf.ed.ac.uk/rbf/HIPR2/log.htm**
* **https://blog.csdn.net/jorg_zhao/article/details/52687448**
* **https://en.wikipedia.org/wiki/Zero_crossing**
* **https://blog.csdn.net/bettyshasha/article/details/51757185**
* **https://blog.csdn.net/u013263891/article/details/83864948**


# 1. Quickstart / Minimal Setup

Launch Matlab and go to the directory that contains the project, and run:

		imgs = my_edge_detector('path/to/imageFile.jpg')

or

	        imgs = prog1('path/to/imageFile.jpg')

The image will be processed with 5 different sigma values and results will be saved to "/output“ folder. 

The output imgs is a cell array with each element as a edge detection output.


# 2. Installation
This project was developed with Matlab IDE. 

Matlab is the only software needed to be installed to run this project.

*No Computer Vision Toolbox is required.

# 3 Project files
ccess_elem.m

my_conv2.m

my_gaussian_kernel.m

my_log_kernel.m

thresholdingImage.m

zero_crossing_evaluator.m

demo.m

my_edge_detector.m

my_laplacian_kernel.m

my_zero_crossing_detector.m

visulization.m

# 4 Usage
A demo program shows an example of using this project. Run:

        demo

The demo program will start and takes "data/lena.jpg" as input, and generate output into the "output" foler. The demo program also show the outputs with imshow() function.


By default the project will use the 2D convolution function provided by matlab. To switch to using the 2D convolution function that I implemented, simply open my_edge_detector.m and change:

        ifUseMyConv2 = false;

to

        ifUseMyConv2 = true;
