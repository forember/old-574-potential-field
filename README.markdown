Just archiving some old projects of mine.

This is the potential fields project from CSCE 574 (Robotics w/ Dr. Rekleitis).
One interesting thing about this code is it is written in [Literate][] C++.
As such, the easiest way to read the main code is
[on my site](https://tachibanatech.com/potential_field.html).

[Literate]: http://literate.zbyedidia.webfactional.com/

This repo is a ROS package. To build, put this in a catkin workspace's `src`
directory under the name `potential_field`. Example download & build script:

```sh
#!/bin/sh
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://ttech.click/old-574-potential-field.git potential_field
cd ..
catkin_make
```
