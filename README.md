# Welcome to the ARViS!

The goal of this repository is to help advance and disseminate automated injection technology to the neuroscience community. While the automated injection platform described here is not an off-the-shelf commercial solution, we have made efforts to make it as easy as possible for any neuroscience lab to build, use, and modify the platform.

The ARViS is designed to perform multiple injections onto the cortex of mice and non-human primates, but it can also be utilized for other animals and other regions in the brain. Some feature highlights are listed below.

# Feature Highlights
- ~50 μm accuracy
- \>100 sequential injections
- 20 sec traveling time per site
- Flexible setting of injection regions
- 7 x 14 mm wide-field imaging

**Please visit our [wiki](https://github.com/nomurshin/ARViS_prepublication/wiki)!** We've provided comprehensive introduction to help you navigate each part of the platform there.

All scripts of the ARViS platform are written as MATLAB scripts, except for SA-UNet and Autostitch.

# How to Install

1. Fork, clone, or copy the repository to your local computer.
2. [Download MATLAB R2021b](https://www.mathworks.com/products/matlab.html).
3. Download the related toolboxes listed below from the MATLAB Add-On Manager.
4. Download [SA-UNet](https://github.com/clguo/SA-UNet) and create a Python environment to execute it.
5. Download [Autostitch](http://matthewalunbrown.com/autostitch/autostitch.html).

# Dependencies

The environment specifications are as follows:

- **Operating System**: Windows 10 Pro Education
- **Processor**: Intel(R) Core(TM) i7-9700K CPU @ 3.60GHz   3.60 GHz
- **Graphics Card**: NVIDIA GeForce RTX 2070 SUPER
- **MATLAB Version**: MATLAB R2021b
  - Computer Vision Toolbox v.10.3
  - Curve Fitting Toolbox v.3.8
  - Image Acquisition Toolbox v.6.7
  - Image Processing Toolbox v.11.6
  - Optimization Toolbox v.9.4
  - Parallel Computing Toolbox v.7.7
  - Signal Processing Toolbox v.9.1
  - Global Optimization Toolbox v.4.6
  - MATLAB Support Package for USB Webcams v.21.2
  - Image Acquisition Toolbox Support Package for GenICam Interface v.21.2
- **Other Dependencies**:
  - [SA-UNet](https://github.com/clguo/SA-UNet) (refer to the original site for more information)
  - [Autostitch](http://matthewalunbrown.com/autostitch/autostitch.html) (refer to the original site for more information)
