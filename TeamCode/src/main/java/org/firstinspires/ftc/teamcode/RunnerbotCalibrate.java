package org.firstinspires.ftc.teamcode;
/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

	import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
	import com.qualcomm.robotcore.eventloop.opmode.Disabled;
	import com.qualcomm.robotcore.hardware.DcMotor;
	import org.lasarobotics.vision.opmode.LinearVisionOpMode;

/**
 * Version history
 *
 * v 0.1	1/6/18 JMR imported from last year's gameday 3 code, adjusted for use with Runnerbot.
 * v 0.11 1/8/18 JMR renamed "Runnerbot Calibrate".
 * v 0.12 1/8/18 JMR made it a LinearVisionOpMode, to test incompatibilities with OpenCV.
*/

@Autonomous(name="Runnerbot Calibrate", group="Test")
//@Disabled
public class RunnerbotCalibrate extends LinearVisionOpMode {

public Runnerbot robot = new Runnerbot(this);
static final double     DRIVE_SPEED             = 0.50;
static final double     TURN_SPEED              = 0.60;
static final double     TURN_RATIO              = 0.80;

@Override
public void runOpMode() throws InterruptedException {
	robot.init(hardwareMap);
	enableExtension(Extensions.BEACON);

	waitForStart(); // Driver presses PLAY button
	String targetJewel = robot.decideTargetJewel("Blue");
  telemetry.addData("Jewel knock decision: ", targetJewel);
	// Step through each leg of the path.
	// Note: Reverse movement is obtained by setting a negative distance (not speed)
	//   Test straight drives and turns.
	//robot.driveStraight(DRIVE_SPEED, 72.0);  // Forward 6'
	//robot.driveStraight(DRIVE_SPEED, -72.0);  // Back 6'
	//robot.turnAngle(TURN_SPEED,  2*Math.PI); // CCW all the way around
	robot.resetEncoderDrive();
	if (targetJewel.equals("left")) {
		robot.turnAngle (Runnerbot.DRIVE_TURN_SPEED, 0.2);
	} else if (targetJewel.equals("right")) {
		robot.turnAngle (Runnerbot.DRIVE_TURN_SPEED, -0.2);
	}
	robot.resetEncoderDrive();
	robot.driveStraight(DRIVE_SPEED, -3.0);  // Backward 1/8 Field tile
	robot.resetEncoderDrive();
	//robot.turnAngle(TURN_SPEED,  0.2); // CCW Jewel knocking angle
	//robot.turnAngle(TURN_SPEED, -2*Math.PI); // CW all the way around
	//robot.encoderDrive(TURN_SPEED, TURN_SPEED, robot.DRIVEWHEEL_SEPARATION*Math.PI,
	//        -robot.DRIVEWHEEL_SEPARATION*Math.PI);  // S2: Full turn right
	//robot.resetEncoderDrive();
	//robot.driveStraight(DRIVE_SPEED, -24);  // S3: Reverse one Field tile.
	//   Test 4 turn cases.
	//robot.encoderDrive(TURN_RATIO*DRIVE_SPEED, DRIVE_SPEED, TURN_RATIO*12, 12);   // forward left
	//robot.encoderDrive(DRIVE_SPEED, TURN_RATIO*DRIVE_SPEED, 12, TURN_RATIO*12);   // forward right
	//robot.encoderDrive(TURN_RATIO*DRIVE_SPEED, DRIVE_SPEED, -TURN_RATIO*12, -12);   // backward left
	//robot.encoderDrive(DRIVE_SPEED, TURN_RATIO*DRIVE_SPEED, -12, -TURN_RATIO*12);   // backward left
	//robot.turnAngleRadiusDrive(TURN_SPEED,  Math.PI/6,  20.0);   // forward left
	//robot.turnAngleRadiusDrive(TURN_SPEED, -Math.PI/6, -20.0);   // forward right
	//robot.turnAngleRadiusDrive(TURN_SPEED, -Math.PI/6,  20.0);   // backward left
	//robot.turnAngleRadiusDrive(TURN_SPEED,  Math.PI/6, -20.0);   // backward right
	}
}

