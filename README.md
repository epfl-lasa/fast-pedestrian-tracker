# fast-pedestrian-tracker
ROS package to detect and track pedestrians from planar laserscans of their legs.

The message definitions for detections and tracks are not provided, but can be easily reverse-engineered from the main node's [source code](./src/node.cpp).


https://user-images.githubusercontent.com/11740105/204297285-65c2f550-1649-48dc-a1ff-8f2b5781fb2f.mp4

The above video shows results of pedestrian tracking with this repository's code. It shows the positions of pedestrians (circles in various colors) which are tracked from a mobile robot (black circle), as well as their velocity estimates (red arrows).

The code is released under the license GNU GPLv3.

Author: David J. Gonon

Contact: david[dot]gonon[at]epfl[dot]ch
