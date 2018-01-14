package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by josh on 11/2/17.
 * Refactored to hardware changes jmr 12/6/17.
 */

@TeleOp(name="Test Arm, Wrist, Claws", group="Test")
public class TestAllButSliderMode extends LinearOpMode {
    // Does not test the slider.
    private DcMotor armMotor1, armMotor2;
    private double clawPosition = 0.5, wristPosition = 1.0;
    private Servo wristServo, clawServo;

    @Override
    public void runOpMode() {
        // Initialization: map the motors and servos. No other initialization.
        armMotor1 = hardwareMap.get(DcMotor.class, "armMotor1");
        armMotor2 = hardwareMap.get(DcMotor.class, "armMotor2");
        wristServo = hardwareMap.get(Servo.class, "gripperWrist");
        clawServo = hardwareMap.get(Servo.class, "gripperClaws");

        telemetry.addData("Status", "Initialized, waiting for match to start.");
        telemetry.update();

        //Wait for the match to start.
        waitForStart();
        //elapsedTime.reset();

        while(opModeIsActive()) {
            // Use: set arm motor powers according to game pad.
            /*
            if(gamepad1.dpad_up && (armMotor1.getCurrentPosition() < 100)) {
                armMotor1.setPower(0.25);
            } else if(gamepad1.dpad_down && (armMotor1.getCurrentPosition() > 0)) {
                armMotor1.setPower(-0.25);
            } else {
                armMotor1.setPower(0.0);
            }*/

            double power = 0.0;
            double armEncoder = armMotor1.getCurrentPosition();

            // Use: set claws and wrist servos positions according to game
            // pad, tempered. Jittered in testing on Friday, and moved.
            // Partial success.
            clawPosition -= gamepad1.right_stick_y * 0.01; // Stick Y is backwards.
            clawPosition = Math.max(Math.min(clawPosition, 1.0), 0.0); // Restrict to 0.0 - 1.0
            clawServo.setPosition(clawPosition);
            // Wrist did something on Friday testing.
            wristPosition -= gamepad1.left_stick_y * 0.002; // Stick Y is backwards.
            wristPosition = Math.max(Math.min(wristPosition, 1.0), 0.6); // Restrict to 0.6 - 1.0
            wristServo.setPosition(wristPosition);

            if(gamepad1.y) { // yellow button, used for nudging the arm.
                // Did nothing at Meet 1.
                // Use: set arm motor powers according to game pad, tempered.
                // TODO: Make these parameters into constants.
                if(armEncoder < -800) { // Need to apply negative power
                    // Encoder = -800, output = 0. Encoder = -1200, output = -0.35
                    power = Math.min(-(armEncoder + 600.0) / 400.0, 1.0) * -0.35;
                } else { //Need to apply positive power
                    // Encoder = -800, output = 0. Encoder = -400, output = 0.35
                    power = Math.min((armEncoder + 600.0) / 200.0, 1.0) * 0.35;
                }
            } else if (gamepad1.x) {
                if(armEncoder < -400.0) // worked at Meet 1, but not to right
                    // position.
                    power = -0.35;
            } else if (gamepad1.b) { // worked at Meet 1, but not to right
                // position.
                if(armEncoder > -800.0)
                   power = 0.35;
            }
            armMotor1.setPower(-power);

            telemetry.addData("Status", "RUNNING");
            telemetry.addData("Encoder Position", armMotor1.getCurrentPosition());
            telemetry.addData("Claw", clawPosition);
            telemetry.addData("Wrist", wristPosition);
            telemetry.update();
        }
    }
}
