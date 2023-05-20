# **SMARTHELMET**
This is a repository for smarthelmet development (sensor fusion of three drowsy driving detection modules - eeg, cv, imu)

## *Smarhelmet 3D Design*

The following images are 3D Design of the 2nd iteration of the development of the smarthelmet. This design uses a _hard hat_ rather than an actual motorcycle helmet. This is design utilizes the rigidness of the hard hat since we are focusing on creating a _testing rig_ to make the _sensor fusion_ work.

![Smarthelmet 3D Design outside look.](https://github.com/sedna08/smarthelmet/blob/0aabe0294ce65127887dae3d877662d1fbc55b5c/Images/outside%20look.png) ![Smarthelmet 3D Design Electrode Placements](https://github.com/sedna08/smarthelmet/blob/facaa2057c392c58edaed59e896b3b34a3dd6490/Images/Electrode%20Placements.png)

## *System Block Diagram*

This is the block diagram of this smarthelmet development. It uses RaspberryPi 3B+ as the controller of the sensors.

![System Block Diagram](https://github.com/sedna08/smarthelmet/blob/a943ae99c82a158bc1b75f1cb6460c6add2b1ed1/Images/System%20Block%20Diagram.png)

## *System Flow*

The following images are the flowcharts of the whole system as well as the individual modules on how they capture data:

### *General System Flow* ---- *EEG (Electroencephalography Module)*            

<img src = "https://github.com/sedna08/smarthelmet/blob/1ae7d97daf8c31438e3fdca77b00d16b2fde7109/Images/System%20General%20Flow.jpg" height = "250"/> ---- <img src = "https://github.com/sedna08/smarthelmet/blob/9951ae5c92985bfb5781eeba2a5df1243dfcb9ec/Images/EEG%20module%20Flow.jpg" height = "250"/>

### 


### *CV (Computer Vision Module)* 
<img src = "https://github.com/sedna08/smarthelmet/blob/9951ae5c92985bfb5781eeba2a5df1243dfcb9ec/Images/CV%20Module%20System%20Flow%20(1).jpg" height = "250"/>
<img src = "https://github.com/sedna08/smarthelmet/blob/9951ae5c92985bfb5781eeba2a5df1243dfcb9ec/Images/CV%20Module%20System%20Flow%20(2).jpg" height = "250"/>

### *IMU (Inertial Measurement Unit Module)*
<img src = "https://github.com/sedna08/smarthelmet/blob/9951ae5c92985bfb5781eeba2a5df1243dfcb9ec/Images/IMU%20Module%20Flow.jpg" height = "250"/>
