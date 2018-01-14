package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by josh on 11/2/17.
 * Refactored to test only the claws jmr 12/6/17. The claws are operated by a
 * servo on the gripper. They are designed to grip either a Relic or a Glyph.
 */

@TeleOp(name="Test Claws Only", group="Test")
public class TestClawsOnlyMode extends LinearOpMode {
    private double clawPosition = 0.5;
    private Servo clawServo;

    @Override
    public void runOpMode() {
        // Initialization: map one servo. No other initialization.
        clawServo = hardwareMap.get(Servo.class, "gripperClaws");

        telemetry.addData("Status", "Initialized, ready to test " +
          "gripper claws.");
        telemetry.update();

        //Wait for the match to start.
        waitForStart();
        //elapsedTime.reset();

        while(opModeIsActive()) {
            double power = 0.0;

            // Use: set claws servo positions according to game
            // pad, tempered. All 3 claws are operated by a single servo.
            clawPosition -= gamepad1.right_stick_y * 0.01; // Stick Y is backwards.
            clawPosition = Math.max(Math.min(clawPosition, 1.0), 0.0); // Restrict to 0.0 - 1.0
            clawServo.setPosition(clawPosition);

            telemetry.addData("Status", "RUNNING");
            telemetry.addData("Claw", clawPosition);
            telemetry.update();
        }
    }
}
