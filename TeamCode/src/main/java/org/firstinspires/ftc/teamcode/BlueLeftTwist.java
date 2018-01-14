package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.opencv.core.Size;

import static org.firstinspires.ftc.teamcode.Sliderbot.ARM_FULL_OUT;

/**
 * Autonomous OpMode for robot in Blue Alliance, starting on right Balancing
 * Stone.
 *
 * Place the robot on the Stone, camera facing the Jewels, arm retracted.
 * Operations:
 *   Look at Jewels, decide which is on the left. Look at Pictograph, decide which column is to
 *     accept the Glyph.
 *   Deploy arm.
 *   Turn to proper side to knock off Opponent Alliance Jewel from the Balancing Stone. Angle to
 *     turn is plus or minus KNOCK_ANGLE, depending on which Jewel is on the left.
 *   Retract arm.
 *   Turn back minus or plus the KNOCK_ANGLE.
 *
 * Version history
 *   v 0.1 jmr 10/28-/17: a different method for knocking the opponent Jewel off: extend arm, and
 *     turn so it knocks the Jewel off, then turn back to original orientation. Then do movements
 *     similar to Blue Right.
 *   v 0.5 jmr 12/7/17: implemented algorithm developed by me and Luciano
 *     Kholos. Assigned this version number to track with Blue Left.
 */
@Autonomous(name="Blue Left Twist", group ="Gameday 2")
//@Disabled
public class BlueLeftTwist extends LinearVisionOpMode {

    private Sliderbot robot = new Sliderbot(this);

    // Driving behavior
    private final double STRAIGHT_SPEED = 0.60;
    private final double TURN_SPEED = 0.60;
    private final double SLOW_STRAFING_SPEED = 0.35;
    private static final double KNOCK_ANGLE =   0.1; // radians
    private double distance2StrafeRight =            0.0;  // right move inches finally decided upon
    private double angle2TwistKnock =       KNOCK_ANGLE;  // This may change

    /*   Vision behavior        */
    // Vuforia identifies the Pictograph, to learn which column of the
    // Cryptobox is the high scoring one.
    // Give time for Vuforia to stare at Pictograph and identify it.
    private final int IMAGE_ID_TIME = 700; // ms
    String imageName = "nothing";
    byte[] colorCcache;

    // Jewel color decision. OpenCV handles this.
    private String targetJewel = "";
    private int frameCount = 0;
    private static final int SAMPLE_NUMBER = 20;
    private static final int CLEAR_MAJORITY = 14;
    private static final double D2SAFETY = 36.0; // inches for left or right Balancing Stone


    // Meet 3: remove or ignore Vuforia and OpenCV capabilities.
    // Meet 3: make Convolutional Neural Net (cnn) model look at Pictograph
    // and left Jewel scene, and decide one of six cases:
    //   Red Jewel left, Left column Pictograph.
    //   Red Jewel left, Center column Pictograph.
    //   Red Jewel left, Right column Pictograph.
    //   Blue Jewel left, Left column Pictograph.
    //   Blue Jewel left, Center column Pictograph.
    //   Blue Jewel left, Right column Pictograph.
    @Override
    public void runOpMode() throws InterruptedException {
        /*            Get robot ready to move.         */
        robot.hwMapDriveMotors();
        robot.initEncodedDrive();
        robot.setDriveStopBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*            Get arm, slider and gripper ready to act.         */
        robot.hwMapArmMotors();
        robot.initArm();
        //robot.hwMapSliderMotor();
        robot.initSlider();
        robot.hwMapServos();
        //robot.initGripper();

        /*      Vuforia identifies the Pictograph.     */
        //  Get an initialized VuforiaLocalizer, and trackable images.
        //VuforiaLocalizer vuforia = robot.initVuforia(); // To do: untested
        //robot.allTrackables = robot.initPictographs(vuforia); // To do:
        // untested
        //   Need to run the Vuforia, get the column, stash it in a variable,
        // then release the camera.
        waitForStart();
        // Implements algorithm worked out by jmr and Luciano Kholos
        // Retract slider and arm	// Done manually on robot placement; no code
        // needed. But
        targetJewel = detectLeftJewelColor (); // See below: this is coded
        // using OpenCV capability. For Meet 2, we won't attempt a correct
        // column determination.

        // Deploy arm, prepare gripper
        //robot.initArm();
        robot.setArmPosition(robot.ARM_RETRACTED);
        robot.initGripper();

        //Raise wrist // not coded
        //Take picture, and analyze it for left Jewel color, ignoring column
        //  information in the Pictograph.
        targetJewel = detectLeftJewelColor(); // See below: this is coded
        // using OpenCV capability. For Meet 3, we need this extended to a
        // function detectLeftJewelColorCorrectCryptoColumn, that returns a
        // color and column combination.
        //Extend slider about halfway
        robot.sliderMoveTimePower(robot.SHORT_TIME, robot.SLIDING_POWER);
        //Drop wrist // not coded
        //Extend slider some more, to knocking position
        robot.sliderMoveTimePower(robot.SHORT_TIME, robot.SLIDING_POWER);
        //Close claws to knocking position // almost closed. Not coded.
        //Drop wrist
         if (targetJewel == "left") {
             angle2TwistKnock = KNOCK_ANGLE;  // positive angle twists left
             robot.turnAngle(TURN_SPEED, angle2TwistKnock);
             //  Raise wrist now?
             //  Turn right to original position
             angle2TwistKnock = -KNOCK_ANGLE;
             robot.turnAngle(TURN_SPEED, angle2TwistKnock);
         }
         else if (targetJewel == "right") {
           angle2TwistKnock = -KNOCK_ANGLE; // negative angle twists right
           robot.turnAngle(TURN_SPEED, angle2TwistKnock);
             //  Raise wrist now?
             //  Turn right to original position
           angle2TwistKnock = KNOCK_ANGLE;
           robot.turnAngle(TURN_SPEED, angle2TwistKnock);
         }
         // else { // no Jewel color decision, so don't twist
        // Raise wrist // or now, after the two turns?

        // Retract slider
        robot.sliderMoveTimePower(robot.MEDIUM_TIME, robot.SLIDING_POWER);
        //Strafe slowly right to Center column. For Meet 2, not attempting a
        //  Glyph placement, we'll be off by plus or minus JEWEL_KNOCK.
        robot.moveSidewaysInches (SLOW_STRAFING_SPEED, D2SAFETY);
    }

    private String detectLeftJewelColor () {
       /*      OpenCV detects the color of the left Jewel.   */
        String colorReport = "";
        String leftColor = "";
        String targetJewel = "";
        int redVotes = 0;
        int blueVotes = 0;
        String voteTally = "";

        //  Wait for OpenCV vision to initialize.
        try { waitForVisionStart(); }
        catch (InterruptedException e) {
            System.err.println("Interrupted Exception waiting for OpenCV: " + e.getMessage());

        }
        this.setCamera(Cameras.PRIMARY);

        /**
         * Setting the width and height of the frame.Larger frame size is
         * sometimes more accurate, but surely slower.
         **/
        this.setFrameSize(new Size(900, 900));

        /**
         Unmodified from Velocity Vortex detection of actively lit sides of
         beacon, Red or Blue.
         */
        enableExtension(Extensions.BEACON);

        /**
         * More thorough methods are slower; faster ones are less reliable.
         */
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        /**
         * Set color tolerances
         * 0 is default, -1 is minimum and 1 is maximum tolerance
         */
        beacon.setColorToleranceRed(0);
        beacon.setColorToleranceBlue(0);

        /**
         * Set analysis boundary
         * You should comment this to use the entire screen and uncomment only if
         * you want faster analysis at the cost of not using the entire frame.
         * This is also particularly useful if you know approximately where the beacon is
         * as this will eliminate parts of the frame which may cause problems
         * This will not work on some methods, such as COMPLEX
         **/
        //beacon.setAnalysisBounds(new Rectangle(new Point(width / 2, height / 2), width - 200, 200));

        //Main loop
        //Camera frames and OpenCV analysis will be delivered to this method as quickly as possible
        //This loop will exit once sufficient color votes are tallied.
        while (frameCount < SAMPLE_NUMBER) {
            //Log a few things
            colorReport = beacon.getAnalysis().getColorString();
            leftColor = colorReport.substring(0,3);

            telemetry.addData("Jewel colors", colorReport);
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            //telemetry.addData("Frame Size", "Width: " + width + " Height: "
            //  + height);
            telemetry.addData("Frame Counter", frameCount);

            // You can access the most recent frame data and modify it here
            // using getFrameRgba() or getFrameGray().
            // Vision will run asynchronously (parallel) to any user code so
            // your programs won't hang.
            // You can use hasNewFrame() to test whether vision has processed a
            // new frame.
            // Once you copy the frame, discard it immediately with
            // discardFrame().
            if (hasNewFrame()) {
                // Get the frame if needed for further processing.
                // Mat rgba = getFrameRgba();
                // Mat gray = getFrameGray();

                // Let the next one be rendered.
                discardFrame();

                //  Custom frame processing here.
                //  For this opmode, get a vote for Red or Blue.
                frameCount++;
                if (leftColor.equals("red")) { redVotes++; };
                if (leftColor.equals("blu")) { blueVotes++; };
            }
            //Wait for a hardware cycle to allow other processes to run
            try {waitOneFullHardwareCycle();} catch (InterruptedException e) {
                System.err.println("Interrupted Exception waiting one " +
                  "hardware cycle: " + e.getMessage());
            }
        }
        // Tally up the votes, decide, report
        if (redVotes >= CLEAR_MAJORITY) { targetJewel = "left"; }
        else if (blueVotes >= CLEAR_MAJORITY) {targetJewel = "right";}
        else { targetJewel = "no decision"; };

        telemetry.addData ("Decision", "turn " + targetJewel + " a little");
        voteTally = "Red: " + redVotes + "    Blue: " + blueVotes;
        telemetry.addData ("Votes", voteTally);
        // Release the camera
        return targetJewel;
    }
}
