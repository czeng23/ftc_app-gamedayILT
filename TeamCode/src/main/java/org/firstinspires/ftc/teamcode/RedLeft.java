package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
 *   Balancing Stone. Distance to strafe is JEWEL_KNOCK.
 *   Strafe to Safety Zone. It will be directly to the right, distance
 *   D2SAFETY. The
 *   distance to strafe will depend on which Jewel was knocked off. It will
 *   be D2SAFETY plus or minus JEWEL_KNOCK.
 *
 * Version history
 *   v 0.1 jmr 10/28/17: just determines color of left Jewel, and makes a
 *     decision accordingly.
 *   v 0.1.1 jmr 10/29/17: added some properties of last year's Sliderbot.
 *   v 0.2 jmr 10/31/17: removed development opmodes, keeping only BlueLeft
 *     and its supporting robot class Sliderbot. The kids will make
 *     BlueRight, RedLeft and RedRight.
 *   v 0.3 jmr 11/17/17: better documentation of how this will lead to our
 *     Meet 1 code.
 *   v 0.4 gf 11/28/17 : partially adapted from blueleft. Adaptation in name only.
 *   v 0.41 jmr 11/29/17, arrives at a correct strafing decision for knocking off the opponent Blue
 *     Jewel from the Platform.
 */
@Autonomous(name="Red Left", group ="Gameday 1")
//@Disabled
public class RedLeft extends LinearVisionOpMode {

    private Sliderbot robot = new Sliderbot(this);

    // Driving behavior
    private final double STRAIGHT_SPEED = 0.60;
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

    // Meet 1: remove or ignore Vuforia and OpenCV capabilities.
    // Meet 1: make Convolutional Neural Net (cnn) model look at Pictograph
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
        //robot.initEncodedDrive(hardwareMap);
        //robot.setDriveStopBehavior(BRAKE);

        /*      Vuforia identifies the Pictograph.     */
        //  Get an initialized VuforiaLocalizer, and trackable images.
        //VuforiaLocalizer vuforia = robot.initVuforia(); // To do: untested
        //robot.allTrackables = robot.initPictographs(vuforia); // To do:
        // untested
        //   Need to run the Vuforia, get the column, stash it in a variable,
        // then release the camera.
        waitForStart();

        targetJewel = detectLeftJewelColor (); // See below: this is coded
        // using OpenCV capability. For Meet 1, we need this extended to a
        // function detectLeftJewelColorCorrectCryptoColumn, that returns a
        // color and column combination.

        // Deploy arm, move according to decision
        //robot.initArm();
        //robot.initClaw();

        // if (targetJewel = "left" {
        //   robot.setArmPosition (ARM_FULL_OUT); // method not implemented
        //   robot.goSideways (-JEWEL_KNOCK); } // negative distance goes left
        //     // robot.goSideways is implemented
        //   robot.setArmPosition (ARM_RETRACTED);
        //   robot.goSideways (D2SAFETY + JEWEL_KNOCK);
        // }
        // else if (targetJewel = "right" {
        //   robot.setArmPosition (ARM_FULL_OUT); // method not implemented
        //   robot.goSideways (JEWEL_KNOCK);
        //   robot.setArmPosition (ARM_RETRACTED);
        //   robot.goSideways (D2SAFETY - JEWEL_KNOCK);
        // } else {
        //   robot.goSideways (D2SAFETY);
        // }
        // // Robot should be sitting in the Safety Zone.
        //
        // For Meet 1, we add here code to drop the Glyph into the correct
        // column. Robot should still be sitting in the Safety Zone.
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
        if (redVotes >= CLEAR_MAJORITY) { targetJewel = "right"; }
        else if (blueVotes >= CLEAR_MAJORITY) {targetJewel = "left";}
        else { targetJewel = "no decision"; };

        telemetry.addData ("Decision", "strafe " + targetJewel);
        voteTally = "Red: " + redVotes + "    Blue: " + blueVotes;
        telemetry.addData ("Votes", voteTally);
        // Release the camera
        return targetJewel;
    }
}
