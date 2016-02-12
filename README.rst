Description
===========

This package allows to convert messages from rgbd-object recognition to naoqi events. In addition, it offers visualisation of the outcome to the robot's tablet (if available).

The notified event contains:

  * object name with its tracked id,
  * position,
  * orientation
  
If a tablet is available, then the recognized object category is visualized. 


Installation
============

The package is dependent on rgbd-object recognition (developped by partners for ROMEO project). 
In case of using tablet, the package should be deployed and launched on the robot; with images from the pics folder copied to /var/www/tmp/.


How it works
============

Launch with roslaunch:

.. code-block:: bash

    roslaunch rgbdobj_tonaoqi rgbdobj_tonaoqi.launch

or launch a binary giving a robot's IP and a port:

.. code-block:: bash

    ./rgbdobj_tonaoqi --pip <ROBOT_IP> --pport 9559

Listen Naoqi events and/or watch the robot's tablet.

