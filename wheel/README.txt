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
controls the starboard side.  Wheel modules 3 and 4 are rotated 180 degrees, so their commands must be inverted.
Command inversion is handled in the roboteqs.

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
seem to have any effect, so instead wrote a mbs script that actually accomplishes that and set it to run on
startup on configs 3, 4
---------------------------------

Wheel 1: Type A, Port Front
Channel 2, Polarity Inverted

Wheel 2: Type B, Starboard Front
Channel 6, Polarity Direct

Wheel 3: Type A, Starboard Back
Channel 6, Polarity Inverted (by script)

Wheel 4: Type B, Port Back
Channel 2, Polarity Inverted (by script)
