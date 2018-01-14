package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.opencv.core.Size;

/**
 * Autonomous OpMode for robot in Blue Alliance, starting on left Balancing
 * Stone.
 *
 * Place the robot on the Stone, camera facing the Jewels, arm retracted.
 * Operations:
 *   Look at Jewels, decide which is on the left.
 *   Deploy arm.
 *   Strafe to proper side to knock off Opponent Alliance Jewel from the
 *   Jewel Platform. Distance to strafe is JEWEL_KNOCK.
 *   Strafe to Safety Zone. It will be directly to the right, distance
 *   D2SAFETY. The
 *   distance to strafe will depend on which Jewel was knocked off. It will
 *   be D2SAFETY plus or minus JEWEL_KNOCK.
 *
 * Version history
 *   v 0.1 jmr 12/13/17: another implementation of the algorithm developed by
 *   me and Luciano Kholos. This supports minimum Jewel knocking hardware: a
 *   simple paddle operated by one servo. This paddle knocks the opponent
 *   Jewel off the Platform by strafing.
 */
@Autonomous(name="Blue Left Twist Paddle", group ="Gameday 3")
//@Disabled
public class BlueLeftTwistPaddle extends LinearVisionOpMode {
    private Sliderbot robot = new Sliderbot(this);

    // Driving behavior
    private final double STRAIGHT_SPEED = 0.60;
    private final double SLOW_STRAFING_SPEED = 0.35;
    private final double TURN_SPEED = 0.60;

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
    private static final double JEWEL_KNOCK = 2.75; // inches

    // Meet 3: remove or ignore Vuforia capabilities.
    // Meet 3: omit Convolutional Neural Net (cnn) model look at Pictograph
    //   and left Jewel scene. Instead, use OpenCV to detect color of left
    //   Jewel.

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

        waitForStart();
        //Retract slider and arm 	// Done manually on robot placement; no
        //  code needed.
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
        if (targetJewel == "left") {
            //Strafe slowly left, enough to knock Red Jewel off.
            // negative distance goes left.
            robot.moveSidewaysInches (SLOW_STRAFING_SPEED, -JEWEL_KNOCK);
        }
        else if (targetJewel == "right") {
            //Strafe slowly right, enough to knock Red Jewel off
            robot.moveSidewaysInches (SLOW_STRAFING_SPEED, JEWEL_KNOCK);
        }
        //Raise wrist // not coded
        //Retract slider
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

        telemetry.addData ("Decision", "strafe " + targetJewel);
        voteTally = "Red: " + redVotes + "    Blue: " + blueVotes;
        telemetry.addData ("Votes", voteTally);
        // Release the camera
        return targetJewel;
    }
}
