package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by josh on 11/2/17.
 * Refactored to test only the slider jmr 12/6/17. The slider can move the
 * gripper straight fore and aft. The arm can move it, too, but does so along
 * an arc.
 * This version uses a Tetrix motor unencoded, unlike Josh's original
 * version. That means the only control on total motor turn angle is dead
 * reckoning time.
 */

@TeleOp(name="Test Slider Only", group="Test")
public class TestSliderOnlyMode extends LinearOpMode {
    private Sliderbot robot = new Sliderbot(this);
    private DcMotor sliderMotor;
    private static final double SLIDING_POWER = 0.35;
    private double power = 0.0;
    private ElapsedTime runtime = new ElapsedTime();
    private static final double SHORT_TIME = 1.0; // seconds
    private static final double MEDIUM_TIME = 2.0; // seconds
    private static final double LONG_TIME = 2.0; // seconds

    @Override
    public void runOpMode() {
        // Initialization: map the slider motor only. No other initialization.
        //sliderMotor = hardwareMap.get(DcMotor.class, "sliderMotor");
        robot.initSlider (); // to do: test this one line, with the one
        // above it commented out.
        telemetry.addData("Status", "Initialized, ready to test slider. ");
        telemetry.update();

        //Wait for the match to start.
        waitForStart();

        if (gamepad1.y) {// yellow button, move FORWARD for a while
            sliderMotor.setPower (SLIDING_POWER);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < SHORT_TIME)) {
                telemetry.addData("Slider", "moving forward: %2.5f s elapsed",
                  runtime.seconds());
                telemetry.update();
            }
            runtime.reset();
        }

        if (gamepad1.a) { // green button, move BACKWARD for a while
            sliderMotor.setPower (-SLIDING_POWER);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < SHORT_TIME)) {
                telemetry.addData("Slider", "moving back: %2.5f s elapsed",
                  runtime.seconds());
                telemetry.update();
            }
            runtime.reset();
        }

        if (gamepad1.x) { // red button, move forward awhile, using
            // Sliderbot robot class method.
            robot.sliderMoveTimePower(robot.SHORT_TIME, robot.SLIDING_POWER);
        }

        if (gamepad1.b) { // blue button, move backward awhile, using
            // Sliderbot robot class method.
            robot.sliderMoveTimePower(SHORT_TIME, -SLIDING_POWER);
        }
    }
}
