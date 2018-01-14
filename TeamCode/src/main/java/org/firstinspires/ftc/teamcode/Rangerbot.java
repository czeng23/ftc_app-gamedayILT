/*
Copyright (c) 2017 Don Bosco Technical Institute Robotics, FTC Team 5197

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted (subject to the limitations in the disclaimer
 below) provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 Neither the name of Don Bosco Technical Institute nor the names of its
 contributors may be used to endorse or promote products derived from this
 software without specific prior written permission.

 NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation
  .VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import java.util.ArrayList;
import java.util.List;

//import com.qualcomm.robotcore.util.TouchSensor; // ** Fixit: Not resolved

/**
 * This is NOT an opmode.
 * <p>
 * This class defines all the specific hardware for a Rangerbot, a simplified
 * version of the Pitsco TETRIX Ranger bot. It is designed to test robot
 * vision by running around and looking at things.
 * <p>
 * Equipment on board:
 *   Two TETRIX motors, unencoded.
 *   Android ZTE phone and phone holder. The phone's camera is the vision
 *   sensor. It can run OpenCV and Vuforia software.
 * <p>
 * Version history
 * v 0.1 jmr 8/16/17 initial Rangerbot class, based on last year's
 * HardwareRangerbot class.
 * v 0.2 jmr 8/17/17 imported some Vuforia code from Sliderbot.
 * v 0.21 jmr 9/5/17 motors brake to dead stop on zero power; they don't coast.
 * Poor: delays too long.
 * JMR v 0.3 12/31/17: Extends GenericRobot class. Last year's code has robot classes that
 *  navigate using Vuforia; that's not present in this version.
 *
 */

public class Rangerbot extends GenericRobot{
  // OpMode members.
  public static final double CAMERA_FROM_FRONT_MM = 220.0; // About 11 inches.
  public static final double SLOW_SPEED = 0.4;
  public static final double FAST_SPEED = 1.0;
  public DcMotor leftMotor = null; // Front wheel drive
  public DcMotor rightMotor = null;
  private LinearOpMode currentOpMode;
  private LinearVisionOpMode currentVisionMode;
  public double knownX;
  public double knownY;
  public double knownHeading;
  public boolean seeking = false;

  /* local robot members. */
  HardwareMap hwMap = null;

/* Constructors */
public Rangerbot() {
  super ();
};
public Rangerbot(LinearOpMode linearOpMode) {
  super (linearOpMode);
  currentOpMode = linearOpMode;
}
public Rangerbot(LinearVisionOpMode seeingOpMode) {
  super (seeingOpMode);
}

  /*************************************************************************
   * Robot initialization methods.
   *************************************************************************/
    /* Register installed equipment according to the configuration file.     */
  public void initFromConfiguration(HardwareMap ahwMap) {
    // Save reference to Hardware map
    hwMap = ahwMap;

    // Define Motors
    leftMotor = hwMap.dcMotor.get("leftMotor");
    rightMotor = hwMap.dcMotor.get("rightMotor");
  }

  public void stopDriveMotors() {
    // Set all motors to zero power
    leftMotor.setPower(0);
    rightMotor.setPower(0);
  }

  /* Initialize standard drive train equipment. */
  public void initDrive() {
    // Left motor is connected to REV Robotics hub motor port 0.
    leftMotor.setDirection(DcMotor.Direction.REVERSE); // AndyMark motor:
      // REVERSE
    rightMotor.setDirection(DcMotor.Direction.FORWARD);// AndyMark motor:
      // FORWARD
    stopDriveMotors();
    ;

    // Set all motors to run without encoders.
    leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    // Set all motors to stop dead, not coast, when power removed.
    //leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    //rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  }

  /****************************************************************************
   * Motor movement methods.
   ***************************************************************************/
    /* Most primitive: tells motors to run at given speeds, for given
    distances.   */
  public void justDrive(double leftSpeed, double rightSpeed) {
    // Go!
    leftMotor.setPower(Math.abs(leftSpeed));
    rightMotor.setPower(Math.abs(rightSpeed));

    // keep looping while we are still active, and any motors are running.
    while (currentOpMode.opModeIsActive() && (leftMotor.isBusy())) {
    }

    stopDriveMotors();

    // Turn off RUN_TO_POSITION
    leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  }

  /****************************************************************************
   * Robot vision members are inherited from GenericRobot class.
   ****************************************************************************/

}
