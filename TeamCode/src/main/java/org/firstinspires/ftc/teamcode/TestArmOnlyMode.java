package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Refactored from josh's 11/2/17 to test only the arm, using Sliderbot robot
 * class members. The arm can move the gripper fore and aft, along a circular
 * arc with high point at midway extension. The slider can move the gripper,
 * too, but does so straight fore and aft.
 */

@TeleOp(name="Test Arm Only", group="Test")
public class TestArmOnlyMode extends LinearOpMode {
    private Sliderbot robot = new Sliderbot(this);

    @Override
    public void runOpMode() {
        // Initialization: map the motors. No other initialization.
        robot.hwMapArmMotors();

        telemetry.addData("Status",
          "Arm initialized, waiting to test arm. ");
        telemetry.update();

        //Wait for testing to start.
        waitForStart();
        //elapsedTime.reset();

        while(opModeIsActive()) {
            // Use: set arm motor powers according to game pad.
            /*
            if(gamepad1.dpad_up && (armMotor1.getCurrentPosition() < 100)) {
                armMotor1.setPower(0.25);
                armMotor2.setPower(-0.25);
            } else if(gamepad1.dpad_down && (armMotor1.getCurrentPosition() > 0)) {
                armMotor1.setPower(-0.25);
                armMotor2.setPower(0.25);
            } else {
                armMotor1.setPower(0.0);
                armMotor2.setPower(0.0);
            }*/

            double power = 0.0;
            double armEncoder = robot.armMotor1.getCurrentPosition();
              // The armMotor2 will just mimic armMotor1.

            if(gamepad1.y) { // yellow button, used for nudging the arm.
                // Another use: set arm motor powers according to game pad,
                // tempered.
                // TODO: Make these parameters into constants.
                if(armEncoder < -800) { // Need to apply negative power
                    // Encoder = -800, output = 0. Encoder = -1200, output = -0.35
                    power = Math.min(-(armEncoder + 600.0) / 400.0, 1.0) * -0.35;
                } else { //Need to apply positive power
                    // Encoder = -800, output = 0. Encoder = -400, output = 0.35
                    power = Math.min((armEncoder + 600.0) / 200.0, 1.0) * 0.35;
                }
            } else if (gamepad1.x) {
                if(armEncoder < -400.0)
                    power = -0.35;
            } else if (gamepad1.b) {
                if(armEncoder > -800.0)
                   power = 0.35;
            }
            robot.armMotor1.setPower(-power);
            robot.armMotor2.setPower(power); // Arm motors face each other.

            telemetry.addData("Status", "RUNNING");
            telemetry.addData("Encoder Position",
              robot.armMotor1.getCurrentPosition());
            telemetry.update();
        }
    }
}
