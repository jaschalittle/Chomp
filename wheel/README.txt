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

Input from the right side of the stick goes to channel 2 on the receiver (so that should go to the left wheels)
Input from the left side of the stick goes to channel 6 on the receiver (so that should go to the right wheels)

Enable/Disable (controlled by "Switch B" on the Futaba) goes to channel 7 on the receiver

L/R input is read by RC1 on the roboteq controller
Enable/Disable is read by RC4 on the roboteq controller

Weird facts:
Channel 2 and Channel 6 have different min/max/centers and thus need different configs
Setting "Capture Polarity" to "Inverted" (via the Roborun UI) on the wheels that need to be inverted doesn't seem to have any effect, so instead wrote a mbs script that actually accomplishes that and set it to run on startup on configs 3, 4
---------------------------------

Wheel 1: Type A, Left Front
Channel 2, Polarity Direct

Wheel 2: Type B, Right Front
Channel 6, Polarity Direct

Wheel 3: Type A, Right Back
Channel 6, Polarity Inverted

Wheel 4: Type B, Left Back
Channel 2, Polarity Inverted


