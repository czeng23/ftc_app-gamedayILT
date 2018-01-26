package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.firstinspires.ftc.robotcontroller.external.samples
				 .ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation
				 .VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Version history
 * v 0.1 jmr 1/26/18: Just test Jewel knocking decision and read Pictograph.
 */

@Autonomous(name = "Test Vision Only", group = "ATest")
//@Disabled
public class TestVisionOnly extends LinearVisionOpMode {
private static final int VUFORIA_GIVEUP = 5;
private Runnerbot robot = new Runnerbot(this);
// Jewel color decision. OpenCV handles this.
private String targetJewel = "no Jewel decision";
private String targetColumn = "no column decision";
private String report = "";
private VuforiaTrackable relicTemplate = null;

@Override
public void runOpMode() throws InterruptedException {
	waitForStart();
	robot.runtime.reset();

	// Using Vuforia, look at the Pictograph to the left of the Jewels. Decide which
	// Cryptobox column is the one coded into that Pictograph.
	VuforiaLocalizer vuforia;
        /*
         * Tell Vuforia the view that we wish to use for camera monitor (on the RC phone). If no
         * camera monitor is desired, use the parameterless constructor instead (commented out in
         * full sample ConceptVuMarkIdentification).
         */
	int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
	VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

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
	VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
	VuforiaTrackable relicTemplate = relicTrackables.get(0);
	relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

	telemetry.addData(">", "Press Play to start.");
	telemetry.update();
	waitForStart();

	relicTrackables.activate();

	while (opModeIsActive()) {

		/**
		 * See if any of the instances of {@link relicTemplate} are currently visible.
		 * {@link RelicRecoveryVuMark} is an enum which can have the following values:
		 * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
		 * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
		 */
		RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

		if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
			telemetry.addData("VuMark", "%s visible", vuMark);
		}
		else {
			telemetry.addData("VuMark", "not visible");
		}

		telemetry.update();
	}

	// Using OpenCV capability, look at Jewels, decide whether the one to be knocked off is
	// on the left or right.
	enableExtension(Extensions.BEACON);
	targetJewel = robot.decideTargetJewel("Blue"); // TESTED in gameday 3,
	report += String.format("Jewel decision %.2f", robot.runtime.seconds());
	report += "s. knocking " + targetJewel;
	telemetry.addData("Decision", report);
	robot.runtime.reset();

	//super.stop();
}
}

