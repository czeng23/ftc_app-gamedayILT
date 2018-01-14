package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by josh on 11/2/17.
 * Refactored to test only the wrist (rotates the gripper) jmr 12/6/17.
 */

@TeleOp(name="Test Wrist Only", group="Test")
public class TestWristOnlyMode extends LinearOpMode {
    private double wristPosition = 1.0;
    private Servo wristServo;

    @Override
    public void runOpMode() {
        // Initialization: map one servo. No other initialization.
        wristServo = hardwareMap.get(Servo.class, "gripperWrist");

        telemetry.addData("Status", "Initialized, ready to test wrist. ");
        telemetry.update();

        //Wait for the match to start.
        waitForStart();
        //elapsedTime.reset();

        while(opModeIsActive()) {
            double power = 0.0;

            // Use: set wrist servo position according to game
            // pad, tempered.
            wristPosition -= gamepad1.left_stick_y * 0.002; // Stick Y is backwards.
            wristPosition = Math.max(Math.min(wristPosition, 1.0), 0.6); // Restrict to 0.6 - 1.0
            wristServo.setPosition(wristPosition);

            telemetry.addData("Status", "RUNNING");
            telemetry.addData("Wrist", wristPosition);
            telemetry.update();
        }
    }
}
