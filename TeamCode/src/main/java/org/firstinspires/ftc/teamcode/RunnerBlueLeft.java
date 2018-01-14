package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Autonomous OpMode for robot in Blue Alliance, starting on left Balancing
 * Stone. This version runs on the Runnerbot, and is used for testing the
 * minimal paddle arm there.
 * <p>
 * Place the robot on the Stone, with right front corner about 5mm behind front rivets on the
 * Stone, and about 5mm to the left of the right side rivets, camera facing the Jewels, paddle
 * retracted.
 * <p>
 * Version history
 * v 0.1 jmr 12/14/17: Another implementation of the algorithm
 * developed by George Flores, Oscar Cendejas,
 * me and Luciano Kholos. This supports minimum Jewel knocking hardware: a
 * simple paddle operated by one servo. This paddle knocks the opponent
 * Jewel off the Platform by strafing. No gripper or claws, no attempt to
 * place Glyphs or a Relic.
 * JMR v 0.2 12/31/17: uses new Runnerbot, an extension of GenericRobot class.
 * JMR v 0.3 1/7/18: can initialize motors and paddle.gl
 * JMR v 0.31 1/7/18: runs paddle correctly; movement crashes on testing opModeIsActive().
 * VD,LK v 0.4 1/10/18: Change the knock angle. JMR: uses consolidated robot method init.
 * VD,LK v 0.41 1/10/18: Change the knock angle, we made a delay between the move and the knock.
 * This one works, probably beta quality for gameday3.
 * JMR v 1.0 1/11/18: Production code for Meet 3. Does knock and run to the Safety Zone. Open CV
 * color limits adjusted to reject brown in the Pictographs. Cleanup code added to
 * Runnerbot:ecoderDrive, to get better transition from one segment to the next.
 */

@Autonomous(name = "Runner Blue Left", group = "Game Day 3") // TESTED, works
  //@Disabled
  public class RunnerBlueLeft extends LinearVisionOpMode {
  private static final double D2SAFETY = 36.0; // inches for left or right Balancing Stone
  private static final double JEWEL_KNOCK_ANGLE = 0.30; // radians
  private static final double JEWEL_KNOCK_DISTANCE = 2.75; // inches;
	private double turn2Safety = -Math.PI/2; // 90 degrees CW
  private Runnerbot robot = new Runnerbot(this);

  // Jewel color decision. OpenCV handles this.
  private String targetJewel = "no Jewel decision";
  private String targetColumn = "no column decision";
  private String report = "";

  @Override
  public void runOpMode() throws InterruptedException {
		/*            Get drive train ready to move.         */
		robot.init(hardwareMap);

		/*            	Get paddle ready to act.      		   */
	  robot.initPaddle();

	  /*								Initialize OpenCV.							 	 */
	  enableExtension(Extensions.BEACON);
  	waitForStart();
	  robot.runtime.reset();
		// Operations:
		// Using OpenCV capability, look at Jewels, decide whether the one to be knocked off is
		// on the left or right. This should also work for RunnerBlueRight.
		targetJewel = robot.decideTargetJewel("Blue"); // TESTED, works.
    report = String.format("Jewel decision %.2f", robot.runtime.seconds());
	  report += "s. knocking " + targetJewel;
	  telemetry.addData("Decision", report);
	  telemetry.update();
	  robot.runtime.reset();

	  robot.paddle.setPosition(Runnerbot.PADDLE_DEPLOYED_POSITION);
		sleep (1000);	// Let paddle drop, before doing the chosen turn. TESTED, works.

	  if (targetJewel.equals("left")) {
		  // Turn counterclockwise, enough to knock Red Jewel off.
		  // Angle to turn is JEWEL_KNOCK. Positive angle goes left.
			robot.turnAngle (Runnerbot.DRIVE_TURN_SPEED, JEWEL_KNOCK_ANGLE);
			// TESTED: works.
			turn2Safety -= JEWEL_KNOCK_ANGLE;
			//robot.turnAngle (Runnerbot.DRIVE_TURN_SPEED, -JEWEL_KNOCK_ANGLE);
	  } else if (targetJewel.equals("right")) {
		  //Turn clockwise, enough to knock Red Jewel off
			robot.turnAngle (Runnerbot.DRIVE_TURN_SPEED, -JEWEL_KNOCK_ANGLE);
			// TESTED: works.
			turn2Safety += JEWEL_KNOCK_ANGLE;
	  }

		// Using Vuforia, look at the Pictograph to the left of the Jewels. Decide which
		// Cryptobox column is the one coded into that Pictograph.
	  /*targetColumn = robot.detectTargetColumn();
	  report = String.format(". Column decision %.2f", robot.runtime.seconds());
	  report += "s. Column " + targetColumn;
	  telemetry.addData("Decision", report);
	  telemetry.update();
	  robot.runtime.reset();
	  */

	  // Pull paddle back so it won't hit our own Jewel. Allow time for that before the next turn.
		robot.paddle.setPosition(Runnerbot.PADDLE_RETRACTED_POSITION);
		sleep(1000);
		// Turn right toward Cryptobox and Safety Zone.
		robot.turnAngle(Runnerbot.DRIVE_TURN_SPEED, turn2Safety);

	  // Drive to Safety Zone. It will be directly to the right, distance
	  // D2SAFETY. Since the Runnerbot turns on its axis to knock the
	  // Jewel, it does not move toward or away from the Safety Zone.
		robot.driveStraight(Runnerbot.DRIVE_STRAIGHT_SPEED, D2SAFETY);
		// To do: adjust distance to drive according to the detected Pictograph.
	  //robot.driveStraight (Runnerbot.DRIVE_STRAIGHT_SPEED, D2SAFETY); // NOT
	  // TESTED.
	  // To do: code the turn toward the Cryptobox, and Glyph placement.
  }
}

