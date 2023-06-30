# MoGlove
Open Sourced Flex Based Mocap Glove

"MoGlove" is an open-source ESP32 based Wireless mocap solution developed for VR/Animation level finger tracking.
MoGlove is the first flex based finger tracking solution with Machined trained pattern recognition to predict common hand gestures and poses based on curliness of fingers. 
MoGlove uses tensorflow lite for microcontrollers, which handles all the recognition on the embedded board.

Zero drift/occulusion
Pure resistive base tracking
Quick Calibration (under 10 sec)


# Spec
- 100Hz+ capture rate
- Hot swappable sensors and gloves
- 400mAh 6hr battery life 1 hr charge
- Wireless
- Adjustable smoothing
- 

# Usage


# Output
Native Blender addon
Currently supports vmd file format used for MMD animations.
Finger grip bone must be added using GripX2 Plugin found in PMXEditor software/

Future support may include Virtual motion capture, which can be linked to VRChat, UE, Unity, etc...
*BVH format does not support native finger tracking capabillity...

# Education
MoGlove also includes user customizable pose assets, as well as a sampling program(linux) to train custom tensorflow models.
MoGlove project is a beginner friendy, to give first dive into machine learning experience with minimum coding skill thanks to elegant library "EloquentML" for microcontrollers (Only 3 lines in total).



# Honourable Inspiration sources
- Mocopi
- Rokoko Glove
- [Somatic](https://www.youtube.com/watch?v=iTj0lcVSIVU&t=613s&ab_channel=ZackFreedman)
- https://www.youtube.com/watch?v=PCBvUHJH8Gw&t=2011s&ab_channel=AnimationPrepStudios

Developed by an single Electrical engineering student/ Inventor
