# Visible Spectrometer Ros Implementation

This code is adapted from Les's Lab in his pyspectrometer2 linked here: https://github.com/leswright1977/PySpectrometer2

Visit Les's Youtube Channel at: https://www.youtube.com/leslaboratory

There is a video on his original project here: https://youtu.be/SCp9T8NKfnM

## Our Version

Our version is based off of this Little Garden Spectrometer from Lao Kang: https://www.ebay.com/itm/166874136724

This spectrometer will diffract light coming from its light slit. the different wavelengths of light that we interpret as colors will diffract by different amounts such that this program can interpret the different intensities as a sort of line and output a 2 by 800 array (resolution of spectrometer dumbed to 800x600, widthxheight, to preserve computational power) containing wavelength data (as floats) in row index 0, and corresponding intensity data (as an integer ranging 0-255) in row index 1.

Other than the scientific backing behind this project, it becomes a simple ros publisher.

Be aware that this package utilizes a custom message from the custom interfaces package defined in this workspace

To simplify runtime I have removed the ability to calibrate the spectrometer using this code. simply calibrate using the original code and use that

### Notes & Commands

Spectrometer Calibration should be performed soon and data pushed

run this node by this command:

``ros2 run spectrometer_publisher spectrometer_node --ros-args -p device:=[PLACE_YOUR_USB_CAMERA_DEVICE#_HERE]``

device argument defaults to 0 when not used

use the command

``v4l2-ctl --list-devices``

to list connected usb cameras, should be soemthing like "/dev/video#" where the # is the argument you want to use

if v4l2-ctl not installed run these commands

```
sudo apt-get update
sudo apt-get install v4l-utils
```