package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.opencv.android.JavaCameraView;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.opencv.core.Size;
import org.opencv.videoio.VideoCapture;

import java.util.List;

import static org.lasarobotics.vision.opmode.VisionOpMode.beacon;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Sliderbot, which runs on a mecanum wheel drive train.
 * <p>
 * This hardware class assumes motor, sensor and servo names have been configured
 * in the robot configuration file.
 *
 * Version history
 * Velocity Vortex version history is in repository ftc_app-5197all2016.
 * JMR v 0.1 10/29/17 Version for Relic Recovery, partially converted from
 *   Velocity Vortex version.
 * JMR v 0.2 12/31/17: extends GenericRobot class.
 */

public class Sliderbot extends GenericRobot {
    /*                 General constants                 */
    private static final double SQRT2 = Math.sqrt(2.0);

    /*                   Timing                             */
    public static final double SHORT_TIME = 1.0; // seconds
    public static final double MEDIUM_TIME = 2.0; // seconds
    public static final double LONG_TIME = 2.0; // seconds

    /*                    Robot hardware                     */
    HardwareMap hwMap = null;

    // To do: Some of these public ones should have accessors.
    /*                   Robot dimensions.                       */
    public static double BOT_WIDTH = 17.8 * 25.4;  // Modified Tile Runner
    public static double BOT_LENGTH = 18.0 * 25.4;  // Modified Tile Runner
    public static final double CAMERA_FROM_FRONT = 220.0; // Millimeters.

    /*                     Driving                     */
    private static final double COUNTS_PER_MOTOR_REV = 1120.0; // 1440 for
    // TETRIX Encoder
    // motor maximum speed is set in motor profile, as of FTC SDK v 3.1.
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.96;   // calibrated
    public static final double DRIVEWHEEL_SEPARATION = 19.4;  // was 19.2
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double DRIVE_TURN_SPEED = 0.5;
    public static final double DRIVE_STRAIGHT_SPEED = 0.6;
    public DcMotor leftFrontMotor = null;   // Motor Port 0 on REV motor hub
    public DcMotor leftRearMotor = null;   // Motor Port 1 on REV motor hub
    public DcMotor rightFrontMotor = null;   // Motor Port 2 on REV motor hub
    public DcMotor rightRearMotor = null;   // Motor Port 3 on REV motor hub

    /*  Minimal Robot arm, no more than a Jewel knocking paddle.  */
    public Servo paddle = null; // Servo Port 5 on REV arm hub.
    public static final double PADDLE_RETRACTED_POSITION = 0.0;
    public static final double PADDLE_DEPLOYED_POSITION = 1.0;

    /*                   Full Robot arm                  */
    public DcMotor armMotor1 = null; // Motor Port 0 on REV arm hub.
    public DcMotor armMotor2 = null; // Motor Port 1 on REV arm hub.
    public static final double ARM_AUTO_SPEED = 0.50;
    public static final int ARM_FULL_OUT = 1000; // Encoder counts to extend
    // arm. To do: calibrate this.
    public static final int ARM_RETRACTED = 0; // Zero encoder counts. To do:
    // calibrate this.

    /*  Slider bar extensions to robot arm. Gripper is mounted at their end. */
    public static final double SLIDING_POWER = 0.35; // motor power
    public DcMotor sliderMotor = null; // Motor port 3 on REV arm hub.
    public static final double SLIDER_AUTO_SPEED = 0.35;

    /*     Wrist turns the gripper claws relative to the slider bars.     */
    public Servo gripperWrist = null;  // Servo Port 0 on REV arm hub
    public static final double WRIST_AUTO_SPEED = 0.50;

    /*      3 claws do the gripping. They can handle a Glyph or a Relic.  */
    public Servo gripperClaws = null;   // Servo port 1 on REV arm hub
    // on REV arm hub
    public static final double CLAW_AUTO_SPEED = 0.50;

    public double knownX;
    public double knownY;
    public double knownHeading;

    /* Constructors */
    private LinearOpMode currentOpMode;
    private LinearVisionOpMode currentVisionMode;
    public Sliderbot() {
        super ();
    };
    public Sliderbot(LinearOpMode linearOpMode) {
        super (linearOpMode);
        currentOpMode = linearOpMode;
    }
    public Sliderbot(LinearVisionOpMode seeingOpMode) {
        super (seeingOpMode);
        currentVisionMode = seeingOpMode; }

    /**************************************************************************
     * Robot initialization methods.
     *************************************************************************/

    /* Get all 4 drive motors from hardware map. */
    public void hwMapDriveMotors () {
        leftFrontMotor = hwMap.dcMotor.get("frontLeft");
        leftRearMotor = hwMap.dcMotor.get("backLeft");
        rightFrontMotor = hwMap.dcMotor.get("frontRight");
        rightRearMotor = hwMap.dcMotor.get("backRight");
    }

    /*  Set directions on all 4 drive motors, enabling mecanum wheel drive. */
    public void setDriveDirections() {
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    /* Initialize all 4 drive motors to some RunMode. */
    public void setDriveRunMode(DcMotor.RunMode someRunMode) {
        leftFrontMotor.setMode(someRunMode);
        leftRearMotor.setMode(someRunMode);
        rightFrontMotor.setMode(someRunMode);
        rightRearMotor.setMode(someRunMode);
    }

    /* Set all 4 drive motors to some behavior when they're told to stop. */
    public void setDriveStopBehavior(DcMotor.ZeroPowerBehavior someBehavior) {
        leftFrontMotor.setZeroPowerBehavior(someBehavior);
        leftRearMotor.setZeroPowerBehavior(someBehavior);
        rightFrontMotor.setZeroPowerBehavior(someBehavior);
        rightRearMotor.setZeroPowerBehavior(someBehavior);
    }

    /* Get both arm motors from hardware map. */
    public void hwMapArmMotors () {
        armMotor1 = hwMap.get(DcMotor.class, "armMotor1");
        armMotor2 = hwMap.get(DcMotor.class, "armMotor2");
    }

    /* Get slider motor from hardware map. */
    public void hwMapSliderMotor () {
        sliderMotor = hwMap.get(DcMotor.class, "sliderMotor");
    }

    /* Get both gripper servos from hardware map. */
    public void hwMapServos() {
        gripperWrist = hwMap.get(Servo.class, "gripperWrist");
        gripperClaws = hwMap.get(Servo.class, "gripperClaws");
    }

    /* Initialize slider motor to some RunMode. */
    public void setSlideRunMode(DcMotor.RunMode someRunMode) {
        sliderMotor.setMode(someRunMode);
    }

    /*  Set both slide motor to some behavior when it's told to stop. */
    public void setSlideStopBehavior(DcMotor.ZeroPowerBehavior someBehavior) {
        sliderMotor.setZeroPowerBehavior(someBehavior);
    }

    /* Initialize standard drive train equipment. */
    public void initUnencodedDrive() {
         // Define and Initialize Motors
        hwMapDriveMotors ();
        setDriveDirections();

        // Call initEncodedDrive if encoders are installed.
        // Generally, Autonomous modes will run encoded, and Driver (TeleOp) modes
        //   will run unencoded.
        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*   Initialize drive equipment to use encoders.  CONVERTED. */
    public void initEncodedDrive() {
        // Define and Initialize Motors
        hwMapDriveMotors ();
        setDriveDirections();
        setDriveRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
     *    Specify speed and end condition for all four motors. Move will stop if
     *    either of two conditions occur:
     *  1) One of the four drive motors gets to the desired position.
     *  2) Driver quits the opmode. CONVERTED.
    */

    public void encoderDrive(double leftFrontSpeed, double rightFrontSpeed,
                             double leftRearSpeed, double rightRearSpeed,
                             double leftFrontInches, double rightFrontInches,
                             double leftRearInches, double rightRearInches) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        //  Discard current encoder positions.
        setDriveStopBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setDriveRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Determine new target positions, and pass to motor controller
        newLeftFrontTarget =  (int) (leftFrontInches * COUNTS_PER_INCH);
        newRightFrontTarget = (int) (rightFrontInches * COUNTS_PER_INCH);
        leftFrontMotor.setTargetPosition(newLeftFrontTarget);
        rightFrontMotor.setTargetPosition(newRightFrontTarget);
        newLeftRearTarget =   (int) (leftRearInches * COUNTS_PER_INCH);
        newRightRearTarget =  (int) (rightRearInches * COUNTS_PER_INCH);
        leftRearMotor.setTargetPosition(newLeftRearTarget);
        rightRearMotor.setTargetPosition(newRightRearTarget);

        // Turn On RUN_TO_POSITION
        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Go!
        leftFrontMotor.setPower(Math.abs(leftFrontSpeed));
        rightFrontMotor.setPower(Math.abs(rightFrontSpeed));
        leftRearMotor.setPower(Math.abs(leftRearSpeed));
        rightRearMotor.setPower(Math.abs(rightRearSpeed));

        // keep looping while we are still active, and both motors are running.
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() &&
                leftRearMotor.isBusy() && rightRearMotor.isBusy() &&
                currentOpMode.opModeIsActive()) {
            //Just let the motors do their thing.
        }
    }

    public void stopDriveMotors(){
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

    public void fullPowerDrive () {
        leftFrontMotor.setPower(1.0);
        leftRearMotor.setPower(1.0);
        rightFrontMotor.setPower(1.0);
        rightFrontMotor.setPower(1.0);
    }

    public void drivetrainPower(double power){
        leftFrontMotor.setPower(power);
        leftRearMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightRearMotor.setPower(power);
    }

    //  Command layer. Human driver issues commands with gamepad. Not tempered.
    public String justDrive (){
        String report = "";
        double leftX = -currentOpMode.gamepad1.left_stick_x;
        double leftY = -currentOpMode.gamepad1.left_stick_y;
        double rightX = -currentOpMode.gamepad1.right_stick_x;
        double robotSpeed = Math.sqrt(leftX * leftX + leftY * leftY);
        double robotAngle = Math.atan2(leftY, -leftX) - Math.PI / 4;
        report = String.format(" Speed: %6.3f ", robotSpeed);
        report += String.format(" Course: %6.3f ", robotAngle);
        double leftFrontSpeed =  robotSpeed * Math.cos(robotAngle) - rightX;
        double rightFrontSpeed = robotSpeed * Math.sin(robotAngle) + rightX;
        double leftRearSpeed =   robotSpeed * Math.sin(robotAngle) - rightX;
        double rightRearSpeed =  robotSpeed * Math.cos(robotAngle) + rightX;

        leftFrontMotor.setPower(leftFrontSpeed);
        rightFrontMotor.setPower(rightFrontSpeed);
        leftRearMotor.setPower(leftRearSpeed);
        rightRearMotor.setPower(rightRearSpeed);

        return report;
    }
    /*
     *  All other movement members are at the command layer.
    */

    //  This one requires no command layer to hardware layer translation.
    //  Just continue going straight.
    public void goStraight(double speed) {
        leftFrontMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        leftRearMotor.setPower(speed);
        rightRearMotor.setPower(speed);
    }

    //   Simple wrapper for encoderDrive. Just go straight a number of inches.
    public void driveStraight(double speed, double inches){
        encoderDrive(speed, speed, speed, speed, inches, inches, inches, inches);
    }

    //  Just continue sliding sideways. Positive speed slides right.
    public void goSideways(double speed) {
        leftFrontMotor.setPower(-speed);
        rightFrontMotor.setPower(speed);
        leftRearMotor.setPower(-speed);
        rightRearMotor.setPower(speed);
    }

    //  Another simple wrapper. Just go sideways a number of inches.
    //    Slips irregularly fore and aft.
    public void moveSidewaysInches (double speed, double inches) {
        encoderDrive (speed, speed, speed, speed, inches, -inches, -inches, inches);
    }

    //   Wheel slippage can cause fore or aft drift.
    public void turnAngle (double speed, double angle) {
        double inches = angle * DRIVEWHEEL_SEPARATION/SQRT2;
        encoderDrive (speed, speed, speed, speed, -inches, inches, -inches, inches);
    }

    /*  Turning movements. All angles are in radians.  ** Convert to Meet 3 build. */
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
    // ** Test on Interleague build.
    //    1:  go straight.
    //    0:  turn axis is left wheel.
    //    -1: turn axis is between drive wheels. Robot turns on own axis.
    public void steerLeft (double speed, double ratio) {
        Range.clip(ratio, -1.0, 1.0);
        leftFrontMotor.setPower(speed * ratio);
        leftRearMotor.setPower(speed * ratio);
        rightFrontMotor.setPower(speed);
        rightRearMotor.setPower(speed);
    }

    //  Right analog of steerLeft.
    public void steerRight(double speed, double ratio) {
        Range.clip(ratio, -1.0, 1.0);
        leftFrontMotor.setPower(speed);
        leftRearMotor.setPower(speed);
        rightFrontMotor.setPower(speed * ratio);
        rightRearMotor.setPower(speed * ratio);
    }

    //  Drive a curved path by making left wheels turn slower and go
    //    shorter path by a factor of ratio. The right wheels will spin
    //    at parameter speed, and travel the full arc.
    public void turnLeft (double speed, double ratio, double arcInches) {
        Range.clip(ratio, -1.0, 1.0);
        encoderDrive(speed*ratio, speed, speed*ratio, speed,
                arcInches*ratio, arcInches, arcInches*ratio, arcInches);
    }

    //  Right analog of turnLeft.
    public void turnRight (double speed, double ratio, double arcInches) {
        Range.clip(ratio, -1.0, 1.0);
        encoderDrive(speed, speed*ratio, speed, speed*ratio,
                arcInches, arcInches*ratio, arcInches, arcInches*ratio);
    }

    /***********************************************************************************
     * Robot vision members come from the GenericRobot class.
     ***********************************************************************************/


/***********************************************************************************
 *          Robot navigation members.
 ***********************************************************************************/
/*  None for Game Day 3. */

/***********************************************************************************
 *          Robot actuator members.
 ***********************************************************************************/
    /*           Initialization             */

    /*  Set up motors to extend and retract arm */
    public void initArm () {
        armMotor1.setDirection(DcMotor.Direction.FORWARD); // They face opposite each other.
        armMotor2.setDirection(DcMotor.Direction.REVERSE);
        armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        armMotor1.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armMotor2.setMode(DcMotor.RunMode.RESET_ENCODERS);
        armMotor1.setTargetPosition(ARM_RETRACTED);
        armMotor2.setTargetPosition(-ARM_RETRACTED);
    }

    /*   Set up slider motor to run slider, with gripper attached, fore and
       aft.
    */
    public void initSlider () {
        sliderMotor = hwMap.get(DcMotor.class, "sliderMotor");
        sliderMotor.setDirection(DcMotor.Direction.FORWARD);
        setSlideRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
    }

    public void initGripper() {
        gripperWrist = hwMap.servo.get ("gripperWrist"); // Port 0 of Arm hub
        gripperClaws = hwMap.servo.get("gripperClaws"); // Port 1 of Arm hub
        gripperClaws.setPosition(1.0);  // Adjust this so it grips the Glyph
    }

    public void initPaddle() {
        paddle = hwMap.servo.get("paddleServo"); // Port 5 of Arm hub
        // paddle.setPosition(PADDLE_RETRACTED_POSITION);
        // This may be omitted if the paddle is manually retracted.
    }

    /*            Usable in Autonomous and in TeleOp macros          */
    public void setArmPosition (int someArmPosition) {
        armMotor1.setPower(ARM_AUTO_SPEED);
        armMotor2.setPower(ARM_AUTO_SPEED);
        armMotor1.setTargetPosition(someArmPosition); // motors face each other
        armMotor2.setTargetPosition(-someArmPosition);
    }

    public void sliderMoveTimePower(double time2Move, double speed) {
        runtime.reset();
        setSlideStopBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderMotor.setPower(speed);

        while (currentOpMode.opModeIsActive() && (runtime.seconds() < time2Move)) {
            // just let motor run until runtime times out.
        }
        runtime.reset();
    }

    public void setWristPosition (double someWristPosition) {
        gripperWrist.setPosition(someWristPosition);
    }

    public void setClawPosition (double someClawPosition) {
        gripperClaws.setPosition (someClawPosition);
    }
}