1.5 10/02/2015
--------------
- Fixed sporadical USART crash.
- Fixed USB LED in FBL2XXX and FDC3XXX.
- Implemented Sin/Cos sensore mode.
- Added CTRIM command.
- FIxed SCRO, CR and BCR commands.
- Implemented Stall Detection when Encoder is disconnected.
- Fixed Hall Speed fluctuation.
- Implemented Hall Sinusoidal mode.
- Fixed DI for CANOpen interface.
- Removed noise on startup for BL motors.
- Support of more than one MagSensors in single Controller.
- Implemented Sensor Offset calculation when idle.
- Implemented Amps Limit during BIND process.
- Adapted default configuration for digital outputs.
- Implemented CANOpen for all supporting boards (STM32F30X).
- Fixed USB Serial Number(board type dependent).
- Implemented FOC.
- Fixed Script download overflow bugs.
- Fixed crash when configuring SPI Sensor Mode.
- Fixed overflow and several bugs in Count Position mode.
- Fixed Overvolt error in F3SDC2XXX.
- Fixed motor squeal when enabling encoders and Encoder Counters in FDC3XXX.
- Fixed crash when configuring to Encoder Sinusoidal in MBL1XXX/SBL1XXX.
- Fixed delayed pulse in capture.
- Fixed Short problem for MDC2XXX rev.3.2
- Implemented MiniJ1939 CAN mode.
- Fixed CANOpen bugs concerning node id 32 and TPDOs 3-4.
- Fixed angle adjust and added stall detection in Encoder Sinusoidal mode
- Fixed Battery Amps for single channel boards with only one sensor.
- Fixed deadtime in PWM in FBL2XXX.
- Implemented unified RoboCAN for all boards, RIOX and Magsensor.

1.4b 03/31/2015
---------------
- MCU overheat error no longer sticks
- Remove Short fault when powering up from power control.
- Fixed Battery Amps for single channel boards
- Added error logs for SDCard
- Fixed Amps averaging on FBL2xxx
- Added Inverted USART support
- Added %sopen, %swrit SDCard functions
- Fixed MOSFETS Off on MDC1xxx, XDC2xxx
- Fixed knocking on FBL2360
- Added support for FBL2360v21
- !AC now works same as in ^MAC
- Progressive Angle Advance in Sinusoidal mode
- Patched Motor Amps sign in Sinusoidal mode
- Fixed CANOpen TPDO Send
- Fixed Short Circuit disable on XDC2xxx
- Added MDC1230
- Removed zero search on Sinusoidal with SPI feedback
- Fixed channel 2 speed capture on FBL2xxx
- Fixed abrupt speed change in Position Count mode
- Fixed negative number of brushless poles in Speed Position, and Position Count
- Sinusoidal with SPI sensor mode
- FDC3xxx support

1.4a 11/21/2014
---------------
- Fixed Count Position bugs
- Relaxed MaxPWM Freq to 30kHz
- Detect Amps calibration loss
- Fixed BEE range
- Added Speed Position mode
- Fixed position count mode bug
- Temperature capture on Analog in for RCB500
- Added continuous count position tracking
- Added script download via RoboCAN
- Added user storage in battery backed RAM
- CPLD Programming works in RS232 mode
- Removed Selftest function

v1.4 09/09/2014
---------------
- Restored CPLD Programming in RS232 mode
- Fixed RoboCAN startup problem
- Fixed CANOpen TPDO and RPDO
- Restructured source files
- Reduced filtering on pulse input to allow encoder capture > 50kHz
- Improved fbl2xxx current sense

v1.4 08/01/2014
---------------
- RoboCAN implementation
- Enable pull ups on RC inputs on XDC2xxx
- Fixed crashing firmware during script exec and motor spin on MBLxxx
- Speed capture is averaged over 10ms on all models. Changed HDC, HBL and MDC

v1.3 05/15/2014
---------------
- Fixed Encoder and Brushless soft limit switches
- Fixed Query History bug
- Added script redirection support to TTL serial port on SDCxxxx
- Added Magsensor Gyroscope in multiPWM
- Improved Short Circuit detection & protection on XDC2xxx and MDC1xxx
- Added SENT protocol
- Added 16-point linearization option on inputs
- Added XDC models
- Added support for MBL1xxx v3.0
- Added filter and FIFO on CAN
- Widen PWM motor drive frequency range
- Fixed clicking on MBL/LBL

v1.3 03/03/2014
---------------
- Added up down selftest
- Added !S direct speed command in closed loop speed mode


v1.3 01/09/2014
---------------
- Fixed difference between ch1 and ch2 in position count mode

v1.3 01/07/2014
---------------
- Fixed Count Position mode when high PPR encoder
- Fixed missed stops in Count Position mode

v1.3 12/12/2013
---------------
- Fixed motor flag error

v1.3 12/04/2013
---------------
- Added MBL1330 support
- Added sinusoidal switching to SBL13xx

v1.3 11/19/2013
---------------
- Encoder sinusoidal beta release

v1.3 10/23/2013
---------------
- Increaded absolute max motor acceleration from 100ms to 10ms to max RPM
- Eliminated MOSFail oversensitivity

v1.3 10/10/2013
---------------
- LDC1450/1460 auto detection
- Added MDC2460 support
- Fixed TPDO and RPDO in CANOpen
- Added CTPS config parameter
- Modified CANOpen EDS
- Added ?FM motor status query

v1.2 08/26/2013
---------------
- Fixed LDC14XX pulse capture on RC3 RC4 RC5 RC6
- Added SBL1XXX V2.0 support

v1.2 08/12/2013
---------------
- Added averaging of Ana 1 and Ana 2 on VIAL1450
- Fixed !m 0 0 crashing on single channel models

v1.2 07/20/2013
---------------
- Fixed Amps capture error on -S versions of dual channel controllers
- Changed SDC2150x to SDC2160x

v1.2 07/10/2013
---------------
- Added immediate script autostart configuration

v1.2 06/12/2013
---------------
- Fixed closed loop speed with BL Hall feedback

v1.2 06/10/2013
---------------
- Added 60V detect and models
- Added Track query for position relative mode

v1.2 06/03/2013
---------------
- Increased Encoder PPR limit to 32000
- Fixed Echo Off on TTL serial port of SDC1130/SDC21XX

v1.2 05/31/2013
---------------
- Added MDC/LDC2260 support

v1.2 05/25/2013
---------------
- Enabled CAN on SBL1360 and MBL1650 (requires wire fix. Disables Din5-6)
- Eliminated SBL/LBL/MBL knocking introduced on 3/15

v1.2 05/12/2013
---------------
- Fixed Overvolt, Unser volt and other fault Shut Off on MBL/LBL/SBL
- Fixed return to zero with Pulse feedback
- Fixed CANOpen heartbeat
- Fixed LED flashing pattern while in CANopen
- Updated CANOpen object dictionary
- Fixed CAN_GO and other commands
- Fixed DOut2 on LBL1350 and MBL1650
- Fixed CANopen on SDC21x0N
- Fixed analog capture on LDC225x0
- Fixed negative BL count on MBL,LBL,SBL
- Fixed PIC query
- Fixing count position mode with negative BL pole number
- Change Hall counter direction when negative pole number
- replaced SBL1350 with SBL1360
- Enabled and fixed count position mode using Hall sensors
- Enabled count position mode on LDC1450, 
- Corrected PWM frequency on LDC22x0

v1.2 02/09/2013
---------------
- Added SBL1350 support
- Corrected wrong MDC2xxx indentification bug
- Fixed Microbasic

v1.2 01/24/2013
---------------
- Gradual powerdown at overtemp. Starting at 70o
- Fixed Ch2 default Closed Loop error detect on SDC2xxx
- Fixed microbasic crash when goto and gosub

v1.2 12/04/2012
---------------
- Fixed startup kick on MBL1650/LBL1350
- Fixed microbasic on MBL1650

v1.2 11/27/2012
---------------
- Added 16-message buffer in RawCAN
- 12K Microbasic scripting Flash on HDC/HBL
- fixe 29-bit CAN identifier bug
- Added Microbasic variable inspections
- Fixed calibration issue on LDC1450, LBL1350 and MBL1650
- Current counter value becomes destination when switching to position count mode


v1.2 10/28/2012
---------------
- Added SDCard file delete. SDIR reports file sizes.
- Added LBL/HBL1650 versions 60V and 72V
- Added LDC1450 versions 30V
- Added MDC/LBC versions 30V and HE
- Make current position be destination when switching to closed loop
- Fixed pulse capture on LBL1350
- Fixed limit switches when in Count Position and Position Relative mode
- Reseting integrator at every closed loop mode changes
- Fixed SDCard watchdog timeout, Read bug
- Changed SDCard output redirection

v1.2 10/05/2012
---------------
- Fixed hardware encoder counter when Pulse input 5 is used
- Fixed RawCAN bug in sdc21x0 and MDC2150

v1.2 09/24/2012
---------------
- Added Ferno configs
- Added script output config parameter. Not used yet
- Added 60V and 72V support to HDC/VDC2400
- Added 60V and 72V support to HBL/VBL2300
- Added new FNB2000 parameters
- Added Integrator anti windup
- Fixed Integrator cap

v1.2 09/03/2012
---------------
- Clear Integrator when limit switchs are activated

v1.2 08/25/2012
---------------
- Added option to "no command change" at power up

v1.2 08/15/2012
---------------
- Fixed potential issue with single channel controllers

v1.2 08/14/2012
---------------
- Fixed Ch1 Hall speed capture on HBL2350
- Fixed speed capture accuracy on HBLxxxx
- Fixed ESTOP and MGO

v1.2 08/06/2012
---------------
- Added Serial Port bit rate change
- Fixed smooth change from open loop to close loop position tracking

v1.2 07/27/2012
---------------
- Added support for MagSensor
- Added Digital Filter on pulse inputs

v1.2 07/23/2012
---------------
- Fixed Microbasic bug on LDCxxx LBLxxx and SDC1130
- Added LBL1350 support

v1.2 05/22/2012
---------------
- Fixed potential bug on LDC1450 and SDC1130

v1.2 05/01/2012
---------------
- Fixed acceleration setting and improved in closed loop position
- Added MOSFail safety
- Removed Analog Mode from default command priorities
- Replaced Uncal LED with RunScript LED in Roborun status (new roborun needed)
- Fixed CAN2 Receive mode
- Improved Close Loop Count Position mode
- Added CAN Support & Autoswitch USB/CAN
- Improved USB communication reliability

v1.2 03/29/2012
---------------
- fixed speed capture in SDCxxxx
- fixed 1s DOut active at boot on MDC2250
- Added Auto-Switch CAN-USB on HDCxxxx/HBLxxxx
- Added !BND Spektrum BIND runtime command
- Added negative Hall sensor speed capture on xBLxxx boards
- Fixed pulse captures on MDC2250
- Fixed encoder speed capture on MDC2250 and HBL/HDC models
- Changed MDC2250 max encoder frequency from 254kHz to 508kHz
- Restored jumper detect for SDC2150
- Changed control loop to 4ms and fixed acceleration ramp on SDC1130 and LDCxxxx models
- Fixed feedback on LDC1450
- Fixed pulse capture problem on SDC1130
- Fixed crash in SDC1130 when changing EMOD configuration
- Added user nonvolatile storage using EE config and EES command
- Fixed microbasic getvalue crash bug
- Working beta of MINICAN and RAWCAN
- Fixed close loop bug on BL controllers at low speed

v1.2 12/28/2011
---------------
- Reliable RTC startup
- Extended safety wdog timeout to 3x
- Fixed Update for microbasic getvalue()
- Added Read/Write to battery backed up RAM
- Change set/get time to single 32-bit seconds counter
- Fixed encoder speed capure bug introduced by previous release

v1.2 12/05/2011
---------------
- Added support for new contoller models

v1.2 10/31/2011
---------------
- Fixed overrange bug when changing !s in mode 3

v1.2 10/19/2011
---------------
- Fixed Encoder counter on single channel SDC2130/50
- improved speed capture accuracy in SDC2130/50

v1.2 09/07/2011
---------------
- Added AIC and PIC queries

v1.2 09/01/2011
---------------
- Added Ramp preload command !RMP

v1.2 08/24/2011
---------------
- Corrected Encoder enable at startup on SDC2130

v1.2 08/12/2011
---------------
- Corrected Short Circuit detect on MDC2250

v1.2 08/12/2011
---------------
- Corrected Short Circuit release bug
- Corrected controller names
- Changed default short circuit threshold to 1

v1.2 08/01/2011
---------------
- Fixed pulse capture on RCIN1 on MDC2250
- Beta RC Output feature on MDC2250

v1.2 07/24/2011
---------------
- Disable Ain4 and Pin2 default in single channel SDC2130

v1.2 07/24/2011
---------------
- Fixed SDC2130 Encoder bug

v1.2 07/15/2011
---------------
- Fixed SDC2130 RC input 5

v1.2 07/09/2011
---------------
- Fixed SDC2130 bug with pulse capture
- Fixed LED status on digital output
- Single channel amps measure on MDC2250 and HDC2450
- Amps lim raise to 150A/300A on HDC2450

v1.2 05/08/2011
---------------
- BL Hall speed capture on HDC


v1.2 03/30/2011
---------------
- Fixed Mixed mode in Closed Loop speed control
- Changed to latest StdPeriph & USB libraries

v1.2 03/20/2011
---------------
- Added single channel support on SDC2130/60
- Improved Encoder capture on single channel SDC2130/50

v1.2 03/06/2011
---------------
- Fixed MicroBasic large script size bug

v1.2 02/20/2011
---------------
- Chained EncPos motions

v1.2 02/08/2011
---------------
- Fixes HDC2450S CPLD reporting

v1.2 02/03/2011
---------------
- Changed Soft Encoder 2 Pinout
- Fixed closed loop error

v1.2 02/02/2011
---------------
- Fixed MBL amps offset

v1.2 01/16/2011
---------------
- Assembly version of softencoder
- Enable/Disable softencoder

v1.2 01/14/2011
---------------
- Speed Capture on SDC2XXX
- Spektrum Radio support added

v1.2 01/05/2011
---------------
- Fixed _DIN getvalue
- Added TTL USART on SDC2XXX
- Dual BLDC Speed fix
- Added softencoder
- BLDC negative speed fix
- SDC2XXX & MDC2XXX pulse capture