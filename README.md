# AVP simple python code to capture hand motion ======================================

## What it does
It connects with the AVP headset in the same network as your computer, and then captures the tracked coordinate frames of your right hand. 

It also converts the coordinate frames into joint angles that corresponds with the ADAPT Hand v2.

## How to use it
1. Download / clone the repo 
2. Wear/setup the avp with a hotspot connecting the headset and your comptuer
3. Change the ip address in 'test_avp_readout.py' to whatever is the ip of the AVP (this is shown when you start the tracking streamer app on the headset)
4. Run test_avp_readout.py

## Packages to install
avp_stream -> pip install avp-stream
scipy -> pip install scipy
numpy -> pip install numpy





# ADAPT Hand 2 controller ======================================


## What it does
Input control signal (motor commands) to real robot hand (ADAPT Hand 2) 

## How to use it
1. Download / clone the repo 
2. Connect  (1) power cable for raspberry Pi in robot hand,  
            (2) power cable for control board of robot hand, turn on the switch
            (3) ethernet cable between robot hand and your computer
3. Check the IP of the raspberry Pi, using command like: 
    ip a
4. ssh connect the robot's raspberry Pi, 
    ssh adapt@10.42.0.xxx
    password: adapt
5. run command on raspberry Pi:   ros2 launch adapt_hand_driver adapt_hand_rpi.launch.py
6. change IP address in script "python real_ADAPT_hand.py", based on your connection IP for raspberry Pi
        self.ros_client = roslibpy.Ros(host='10.42.0.XXX', port=9090)
7. run in local terminal: python real_ADAPT_hand.py

## Packages to install
roslibpy -> pip install roslibpy