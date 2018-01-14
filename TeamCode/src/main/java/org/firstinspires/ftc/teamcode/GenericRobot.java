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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.detection.objects.Rectangle;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.opencv.core.Point;
import org.opencv.core.Size;

import static org.lasarobotics.vision.opmode.VisionOpMode.beacon;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the hardware for a generic robot. It's mostly vision.
 * Actual development and competition robot classes will extend this one.
 * <p>
 * This hardware class makes no assumption about the configuration file, except the REV Robotics
 * expansion hub.
 *
 * Version history
 * JMR v 0.1 12/31/17 Vision only. Robot lastLocation not used.
 * JMR v 0.2 12/31/17 Usable by Runnerbot and Sliderbot.
 * JMR v 1.0 1/2/18. Production quality. Improved Jewel detection.
 */

public class GenericRobot {
    /*                 General constants                 */
    private static final double SQRT2 = Math.sqrt(2.0);

    /*                        Time                           */
    public ElapsedTime runtime = new ElapsedTime();

// To do: Some of these public ones should have accessors.
    /*                   Robot dimensions.                       */
    public static double BOT_WIDTH = 17.8 * 25.4;  // Modified Tile Runner
    public static double BOT_LENGTH = 18.0 * 25.4;  // Modified Tile Runner

    /*                  Robot vision: OpenCV                        */
    private LinearOpMode currentOpMode;
    private LinearVisionOpMode currentVisionMode;
    public OpenGLMatrix lastLocation = null;

    /* Constructors */
    public GenericRobot() {};
    public GenericRobot(LinearOpMode linearOpMode) {
        currentOpMode = linearOpMode;
    }
    public GenericRobot(LinearVisionOpMode seeingOpMode) {
        currentVisionMode = seeingOpMode; }

    /**************************************************************************
     * Robot initialization methods.
     *************************************************************************/



    /***********************************************************************************
     * Robot vision members.
     ***********************************************************************************/

    public VuforiaTrackable initVuforia (String someTrackableAsset) {
        VuforiaLocalizer vuforia;
      /*
        * Tell Vuforia the view that we wish to use for camera monitor (on the RC phone). If no
        * camera monitor is desired, use the parameterless constructor instead (commented out in
        * full sample ConceptVuMarkIdentification).
        */
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
        //("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters ();

        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters
        //                                            (cameraMonitorViewId);
        parameters.vuforiaLicenseKey =
          "ASkv3nr/////AAAAGYZ9CexhH0K0lDbV090F719DkwXCIXEUmExgnQNDFGjrDrk" +
            "VJnU7xNhuKHLsC32Pb1jmr+6vp6JtpVKvNmTf28ZYkUphDeajNPCLgGVxLjD6xs" +
            "fgBayqSO9bfQFeGkrdEgXlP+2oaz234afhWti9Jn8k71mzbQ4W2koX9yBMWz0YL" +
            "zUWClcasxi6Nty7SUvV+gaq3CzpKVtjKk+2EwV6ibIc0V47LAeB0lDGsGkSzuJ+" +
            "93/Ulpoj+Lwr/jbI2mu/Bs2W7U9mw73CMxvDix9o1FxyPNablla4W5C5lUDm0j2" +
            "lW5gsUNOhgvlWKQ+eCu9IBp53WbW5nfNzhXPaDDh/IlBbZuAMIJuMDEHI5PVLKT9L";
      /*
       * We also indicate which camera on the RC that we wish to use. Here we chose the back
       * (HiRes) camera (for greater range), but for a competition robot, the front camera
       * might be more convenient.
       */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one
         * template, but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset(someTrackableAsset);
        relicTrackables.activate();
        return relicTrackables.get(0);
    }

    //   Relic Recovery specific. Returns which column of the Cryptobox should receive the Glyph
    // mounted in Autonomous, as determined by the Pictograph.
    public String detectTargetColumn() {
        VuforiaTrackable relicTemplate = initVuforia("RelicVuMark");
        RelicRecoveryVuMark vuMark;
        int tries = 0;
        final int VUFORIA_GIVEUP = 5;

        do {
        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */
            tries ++;
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        } while ((vuMark == RelicRecoveryVuMark.UNKNOWN) && (tries < VUFORIA_GIVEUP)) ;
        // Found an instance of the template, or took too many tries.
        return vuMark.toString();
    }

    //   Relic Recovery specific. Returns which Jewel is to be knocked off,
    // the one on the left or right of the Platform.
    public String decideTargetJewel(String alliance) {
       /*      OpenCV detects the color of the left Jewel.   */
        String colorReport = "";
        String leftColor = "";
        String targetJewel = "";
        int frameCount = 0;
        int redVotes = 0;
        int blueVotes = 0;
        String voteTally = "";
        final int SAMPLE_NUMBER = 10;
        final int CLEAR_MAJORITY = 7;
        final int FRAME_WIDTH = 250; // was 900. Minimum is 250 for Nexus 5, 160 for ZTE 9130.
        final int FRAME_HEIGHT = 145; // was 900. Minimum is 145 for Nexus 5, 125 for ZTE 9130.

        //  Wait for OpenCV vision to initialize.
        try { currentVisionMode.waitForVisionStart(); }
        catch (InterruptedException e) {
            System.err.println("Interrupted Exception waiting for OpenCV: " + e.getMessage());
        }
        currentVisionMode.setCamera(Cameras.PRIMARY);
        currentVisionMode.setFrameSize(new Size(FRAME_WIDTH, FRAME_HEIGHT));

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


        //Main loop
        //Camera frames and OpenCV analysis will be delivered to this method as quickly as possible
        //This loop will exit once sufficient color votes are tallied.
        while (frameCount < SAMPLE_NUMBER) {
            //Log a few things
            colorReport = beacon.getAnalysis().getColorString();
            leftColor = colorReport.substring(0,3);

            if (currentVisionMode.hasNewFrame()) {

                // Let the next one be rendered.
                currentVisionMode.discardFrame();

                //  For this opmode, get a vote for Red or Blue.
                frameCount++;
                if (leftColor.equals("red")) { redVotes++; }
                if (leftColor.equals("blu")) { blueVotes++; }
            }
            //Wait for a hardware cycle to allow other processes to run
            try {currentVisionMode.waitOneFullHardwareCycle();} catch
              (InterruptedException e) {
                System.err.println("Interrupted Exception waiting one " +
                  "hardware cycle: " + e.getMessage());
            }
        }
        // Tally up the votes, decide, report
        if (redVotes >= CLEAR_MAJORITY) {
            targetJewel = (alliance.equals( "Blue") ? "left" : "right"); }
        else if (blueVotes >= CLEAR_MAJORITY) {
            targetJewel = (alliance.equals( "Red") ? "left" : "right"); }
        else { targetJewel = "no decision"; }

        // Release the camera
        currentVisionMode.discardFrame();
        return targetJewel;
    }
}