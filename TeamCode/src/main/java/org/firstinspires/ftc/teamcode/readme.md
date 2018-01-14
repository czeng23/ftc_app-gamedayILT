# GearHead TeamCode Module

Welcome!

TeamCode is the module where Team 5197 "the GearHeads" write or paste code for
their robot controller App.  

### Autonomous OpModes
Choose one of these at the beginning of the match, according to which Alliance you are playing
for, and on which Balancing Stone you begin. There are 4 cases:

* Blue Left: Blue Alliance, robot on left Balancing Stone.
* Blue Right: Blue Alliance, robot on right Balancing Stone.
* Red Left: Red Alliance, robot on left Balancing Stone.
* Red Right: Red Alliance, robot on right Balancing Stone.

All these Autonomous modes operate by dead reckoning, or "open loop". That means the robot runs
its path blindly, with no attempt to check its position or orientation, with no feedback to
correct for errors. Obviously, this means the robot must be carefully calibrated before running
any of these. See last year's code for vision-assisted Autonomous navigation; that code closes the
loop.

### TeleOp modes
Run one of the TeleOp modes during the Driver controlled phase of a match.
These modes allow control of the robot using the gamepad. Check the documentation for each one,
to learn which robot functions are controlled by which stick, button or lever on the gamepad.

## Creating your own OpModes

The process for adding OpModes is straightforward. The easiest way to create your own OpMode is to copy a Sample OpMode and make
 it your own. 

Sample opmodes exist in the FtcRobotController module. To locate these 
samples, find the FtcRobotController module in the "Project/Android" tab. 

Expand the following tree elements:
  FtcRobotController / java / org.firstinspires.ftc.robotcontroller / external
  / samples 

A range of different samples classes can be seen in that folder. The class
names follow a naming convention which indicates the purpose of each class. 
The full description of this convention is found in the 
samples/sample_convention.md file.   

A brief synopsis of the naming convention is given here:
The prefix of the name will be one of the following:

* Basic:    This is a minimally functional OpMode used to illustrate the 
skeleton/structure of a particular style of OpMode. These are bare bones 
examples.  
* Sensor:   This is a Sample OpMode that shows how to use a specific sensor. 
It is not intended as a functioning robot, it is simply showing the minimal 
code required to read and display the sensor values.
* Hardware: This is not an actual OpMode, but a helper class that is used to 
describe one particular robot's hardware devices: eg: for a Pushbot. Look at 
any Pushbot sample to see how this can be used in an OpMode. Teams can copy 
one of these to create their own robot definition. 
* Pushbot:  This is a Sample OpMode that uses the Pushbot robot structure as 
a base. This is the class on which our Rangerbot class is based.
* Concept:	This is a sample OpMode that illustrates performing a specific 
function or concept. These may be complex, but their operation should be 
explained clearly in the comments, or the header should reference an external
doc, guide or tutorial.  
* Library:  This is a class, or set of classes used to implement some 
strategy. These will typically NOT implement a full OpMode. Instead they will
be included by an OpMode to provide some stand-alone capability. 

Once you are familiar with the range of samples available, you can choose one
to be the basis for another of our own robots, like the Runnerbot. In all 
cases, the desired sample(s) needs to be copied into your TeamCode module to 
be used. 

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.
 2) Right click on the sample class and select "Copy"
 3) Expand the  TeamCode / java folder
 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"
 5) You will be prompted for a class name for the copy. Choose something 
 meaningful based on the purpose of this class. Start with a capital letter, 
 and remember that there may be more similar classes later.  

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed 
on the Driver Station's OpMode list. Enabling is done by removing or 
commenting out the @Disabled annotation. Here's how to do that:

Each OpMode sample class begins with several lines of code like the ones 
shown below: 

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by
 the code: ``name="Template: Linear OpMode"`` You can change what appears 
 between the quotes to better describe your opmode. 
The "group=" portion of the code can be used to help organize your list of 
OpModes. As shown, the current OpMode will NOT appear on the driver station's
OpMode list because of the ``@Disabled`` annotation which has been included. 
This line can simply be deleted, or commented out, to make the OpMode visible
and enabled.


## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to
 all share a common code organization, with each being able to *see* the  
 others code but each having their own team module with their own code that 
 they maintain themselves. 

In this situation, you might wish to clone the TeamCode module, once for each
 of these teams. Each of the clones would then appear along side each other 
 in the Android Studio module list, together with the FtcRobotController 
 module (and the original TeamCode module). 

Selective Team phones can then be programmed by selecting the desired Module 
from the pulldown list prior to clicking to the green Run arrow.

**Warning:  This is not for the inexperienced Software developer.**
You will need to be comfortable with File manipulations and managing Android 
Studio Modules. These changes are performed OUTSIDE of Android Studios, so 
close Android Studios before you do this. 
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team". This is 
intentional. 

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".
2)  In the new Team0417 folder, delete the TeamCode.iml file.
3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".
4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file,
 change the line that contains package="org.firstinspires.ftc.teamcode" to be
  package="org.firstinspires.ftc.team0417" 
5)  Add: include ':Team0417' to the "/settings.gradle" file.
6)  Open up Android Studios and clean out any old files by using the menu to 
"Build/Clean Project".

= = = = = = = =
To do

o Get FTC SDK to version 3.6.
o Upgrade Vuforia to version 7.