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
 * JMR 1/24/18 v 0.2 Simplified to respond only to Driver 2's game pad right stick, y motion.
 */

@TeleOp(name="Test Arm Only", group="Test")
public class TestArmOnlyMode extends LinearOpMode {
    private Sliderbot robot = new Sliderbot(this);
    @Override
    public void runOpMode() {

        // Initialization: map the arm motors. No other initialization.
        robot.hwMapArmMotors();

        telemetry.addData("Status",
          "Arm initialized, waiting to test arm. ");
        telemetry.update();

        //Wait for testing to start.
        waitForStart();
        //elapsedTime.reset();
        double power = 0.0;

        while(opModeIsActive()) {
            // Use: set arm motor powers according to game pad.
            power = gamepad2.right_stick_y;

            robot.armMotor1.setPower(-power);
            robot.armMotor2.setPower(power); // Arm motors face each other.

            telemetry.addData("Status", "RUNNING");
            telemetry.addData("Encoder Position", robot.armMotor1.getCurrentPosition());
            telemetry.update();
        }
    }
}
