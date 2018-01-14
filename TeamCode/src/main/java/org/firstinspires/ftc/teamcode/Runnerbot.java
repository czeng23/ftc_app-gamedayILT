package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.VisionOpMode;
import org.opencv.core.Size;

import java.util.List;
import java.util.Set;

import static org.lasarobotics.vision.opmode.VisionOpMode.beacon;

/**
 * This is NOT an opmode.
 * <p>
 * This class defines all the specific hardware for a single robot of the
 * Runnerbot class, a modified AndyMark TileRunner. It has a West Coast drive
 * train, with a pair of motors for each side. Each pair is treated as one
 * motor.
 * <p>
 * This hardware class assumes motor, sensor and servo names have been configured
 * in the robot configuration file.
 *
 * Version history
 * Velocity Vortex version history is in repository ftc_app-5197all2016.
 * JMR v 0.1 10/29/17 Version for Relic Recovery, partially converted from
 *   Velocity Vortex version.
 * JMR v 0.2 12/16/17: converted from Sliderbot class, and a method
 *   decideTargetJewel was added.
 * JMR v 0.3 12/31/17: extends GenericRobot class.
 * JMR v 1.0 1/2/18. Production quality. Uses improved GenericRobot class.
 * JMR v 1.1 1/7/18. Fixes v 1.0 bug, by which RunnerBlueLeft could not initialize motors.
 */

public class Runnerbot extends GenericRobot {

    /*                    Robot hardware                     */
    HardwareMap hwMap = null;
    public DcMotor leftMotor = null;   // Motor Port 0 on REV motor hub
    public DcMotor rightMotor = null;   // Motor Port 1 on REV motor hub
    /*                        Time                           */
    public ElapsedTime runtime = new ElapsedTime();

// To do: Some of these public ones should have accessors.
    /*                   Robot dimensions.                       */
    public static double BOT_WIDTH = 17.8 * 25.4;  // Modified Tile Runner
    public static double BOT_LENGTH = 18.0 * 25.4;  // Modified Tile Runner

    /*                     Driving                     */
    private static final double COUNTS_PER_MOTOR_REV    = 1120.0; // 1440 for
    // TETRIX Encoder
    // motor maximum speed is set in motor profile, as of FTC SDK v 3.1.
    private static final double DRIVE_GEAR_REDUCTION    = 1.0;
    private static final double WHEEL_DIAMETER_INCHES   = 4.10; // CALIBRATED

    public static final double DRIVEWHEEL_SEPARATION    = 15.8; // CALIBRATED.
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV *
      DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double DRIVE_TURN_SPEED = 0.25;
    public static final double DRIVE_STRAIGHT_SPEED = 0.25;


    /*  Minimal Robot arm, no more than a Jewel knocking paddle.  */
    public Servo paddle = null; // Servo Port 5 on REV arm hub.
    public static final double PADDLE_RETRACTED_POSITION = 1.0;
    public static final double PADDLE_DEPLOYED_POSITION = 0.0;

    /*                  Robot vision: OpenCV                        */
    private LinearOpMode currentOpMode;
    public OpenGLMatrix lastLocation = null;

    /* Constructors */
    public Runnerbot() {
        super ();
    };
    public Runnerbot(LinearOpMode linearOpMode) {
        super (linearOpMode);
        currentOpMode = linearOpMode;
    }
    public Runnerbot(LinearVisionOpMode seeingOpMode) {
        super (seeingOpMode);
    }

    /**************************************************************************
     * Robot initialization methods.
     *************************************************************************/

    /* Get both drive motors from hardware map. */

    public void initMotors(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor= hwMap.get(DcMotor.class, "left");
        rightMotor = hwMap.get(DcMotor.class, "right");
    }

    /*  Set directions on both drive motors, enabling mecanum wheel drive. */
    public void setDriveDirections() {
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    /* Initialize both drive motors to some RunMode. */
    public void setDriveRunMode(DcMotor.RunMode someRunMode) {
        leftMotor.setMode(someRunMode);
        rightMotor.setMode(someRunMode);
    }

    /* Set both drive motors to some behavior when they're told to stop. */
    public void setDriveStopBehavior(DcMotor.ZeroPowerBehavior someBehavior) {
        leftMotor.setZeroPowerBehavior(someBehavior);
        rightMotor.setZeroPowerBehavior(someBehavior);
    }

    /* Initialize standard drive train equipment. */
    public void initUnencodedDrive() {
         // Define and Initialize Motors
        initMotors (hwMap);
        setDriveDirections();

        // Call initEncodedDrive if encoders are installed.
        // Generally, Autonomous modes will run encoded, and Driver (TeleOp) modes
        //   will run unencoded.
        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*      Initialize drive equipment to use encoders.         */
    public void initEncodedDrive() {
        // Define and Initialize Motors
        initMotors (hwMap);
        setDriveDirections();
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*              Grand Autonomous Encoded Initializer        */
    public void init (HardwareMap someMap) {
        hwMap = someMap;
        initMotors(hwMap);
        // Stop all motors
        setDriveStopBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void resetEncoderDrive() {
        // Zero the encoder counts targets. Side effect is to remove
        //   power, but we will do that explicitly.
        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**************************************************************************
     * Robot movement members.
     **************************************************************************/

    /*
     *  Hardware primitives layer, the most primitive movement members.
     *  These send direct commands to the hardware.
     */

    /*  General movement. Most other movement methods will be wrappers for this.
     *    Specify speed and end condition for both motor pairs. Move will
     *    stop if either of two conditions occur:
     *  1) One of the two drive motor pairs gets to the desired position.
     *  2) Driver quits the opmode.
    */

    public void encoderDrive(double leftSpeed, double rightSpeed,
                             double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        //  Discard current encoder positions.
        setDriveStopBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target positions, and pass to motor controller.
        newLeftTarget =  (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = (int) (-rightInches * COUNTS_PER_INCH);
        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Go!
        leftMotor.setPower(Math.abs(leftSpeed));
        rightMotor.setPower(Math.abs(rightSpeed));

        // keep looping while we are still active, and both motors are running.
        while (leftMotor.isBusy() && rightMotor.isBusy()) {
            // Wait until motors done before doing anything else.
        }
        // Clean up, prepare for next segment.
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stopDriveMotors(){
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void fullPowerDrive () {
        leftMotor.setPower(1.0);
        rightMotor.setPower(1.0);
    }

    public void drivetrainPower(double power){
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    //  Command layer. Human driver issues commands with gamepad.
    public void justDrive (){
        //  Tank drive with the two sticks.
        double leftSpeed = -currentOpMode.gamepad1.left_stick_y;
        double rightSpeed = -currentOpMode.gamepad1.right_stick_y;
        //  Spin on axis with right sick, x motion.
        double rightX = -currentOpMode.gamepad1.right_stick_x;
        //  Not tempered.
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);
    }

    /*
     *  All other movement members are at the command layer.
    */

    //  This one requires no command layer to hardware layer translation.
    //  Just continue going straight.
    public void continueStraight(double speed) {
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
    }

    //   Simple wrapper for encoderDrive. Just go straight a number of inches.
    public void driveStraight(double speed, double inches){
        encoderDrive(speed, speed, inches, inches);
    }

    //   Turn on axis, as though with left and right tank drive joysticks in equal but
    // opposite deflection.
    public void turnAngle (double speed, double angle) { // angle in radians
        double inches = angle * DRIVEWHEEL_SEPARATION/2;
        encoderDrive (speed, speed, -inches, inches);
    }

    /*  Turning movements. All angles are in radians. */
    //  Turn at speed through an angle, with a given radius.
    public void turnAngleRadiusDrive(double speed, double angle, double radius) {

        // One or both turning arcs could be negative.
      // ** implement as wrapper for encoderDrive
        // Degenerate cases: angle = 0, R = 0, R = d/2, R = +infinity
        // (straight drive).
    }

    //    Wrapper for turnAngleRadius
    // ** make this a wrapper for encoderDrive instead.
    public void turnArcRadiusDrive(double speed, double arc, double radius) {
        double targetAngle = arc / radius;
        turnAngleRadiusDrive(speed, targetAngle, radius);
    }

    //  Begin a left turn at speed, sharpness of turn decided by ratio.
    // ** Test on Meet 3 build.
    //    1:  go straight.
    //    0:  turn axis is left wheel.
    //    -1: turn axis is between drive wheels. Robot turns on own axis.
    public void steerLeft (double speed, double ratio) {
        Range.clip(ratio, -1.0, 1.0);
        leftMotor.setPower(speed * ratio);
        rightMotor.setPower(speed);
    }

    //  Right analog of steerLeft.
    public void steerRight(double speed, double ratio) {
        Range.clip(ratio, -1.0, 1.0);
        leftMotor.setPower(speed);
        rightMotor.setPower(speed * ratio);
    }

    //  Drive a curved path by making left wheels turn slower and go
    //    shorter path by a factor of ratio. The right wheels will spin
    //    at parameter speed, and travel the full arc.
    public void turnLeft (double speed, double ratio, double arcInches) {
        Range.clip(ratio, -1.0, 1.0);
        encoderDrive(speed*ratio, speed,
                arcInches*ratio, arcInches);
    }

    //  Right analog of turnLeft.
    public void turnRight (double speed, double ratio, double arcInches) {
        Range.clip(ratio, -1.0, 1.0);
        encoderDrive(speed, speed*ratio,
                arcInches, arcInches*ratio);
    }

    /***********************************************************************************
     * Robot vision members are inherited from the GenericRobot class.
     ***********************************************************************************/

/***********************************************************************************
 *          Robot navigation members.
 ***********************************************************************************/
/*  None for Game Day 3. */

/***********************************************************************************
 *          Robot actuator members.
 ***********************************************************************************/

    /*           Initialization             */
    public void initPaddle() {
        paddle = hwMap.servo.get("Paddle"); // Port 5 of Arm hub
        paddle.setPosition(PADDLE_RETRACTED_POSITION);
        // This may be omitted if the paddle is manually retracted.
    }
}