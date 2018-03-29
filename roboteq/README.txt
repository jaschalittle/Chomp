********Firmware******
MBL1660 controllers should be upgraded to version 1.8d


********Loading wheel configurations********

Connect your computer to the controller via USB
Fire up Roborun+
On the "Configuration" tab hit "Load Profile from Disk"
Select the wheel config for the wheel # you're loading
Hit "Save To Controller"
Go to the scripting tab
Hit open > load up "invert_pulse.mbs"
Hit "Download to Device"

********Wheel Configuration Notes********

The robot is using elevon mixing, which outputs on Ch1 and Ch2.   Ch1 control the port wheel modules, and Ch2
controls the starboard side.  

All wheel modules should rotate clockwise as viewed from the wheel side, with a positive (greater than 1.5ms pulse).
Therefore, one side of the robot should be inverted, if we want to use the same sense on both RC channels. 

Futaba setup:
  Elevon mixing:
    Ail: 50% (increase this for more aggressive turning rates)
    Ele: 100%
    Diff: 0%
  Reverse Ch2

Enable/Disable (controlled by "Switch B" on the Futaba) goes to channel 7 on the receiver

Driving input is read by RC1 on the roboteq controller
Enable/Disable is read by RC4 on the roboteq controller

Weird facts:
Setting "Capture Polarity" to "Inverted" (via the Roborun UI) on the wheels that need to be inverted doesn't
seem to have any effect.  Setting one of the unused digital inputs to Active Low, and its action to: invert direction
does however.
---------------------------------

Wheel 1: Type A, Port Front
Channel 2, Polarity Inverted (Din5)

Wheel 2: Type B, Starboard Front
Channel 6, Polarity Direct

Wheel 3: Type A, Starboard Back
Channel 6, Polarity Direct

Wheel 4: Type B, Port Back
Channel 2, Polarity Inverted (Din5)
