package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OpMode moves Glyph pusher to one of two positions, RETRACT_POS and PUSHOUT_POS, respectively
 * on game pad buttons Y and A.
 * The code is structured as a LinearOpMode.
 *
 * This code assumes a Servo configured with the name "Pusher" as is found on Runnerbot.
 * v 0.1 Refactored 1/25/18 jmr from paddle test opmode for Runnerbot. Tests only the minimum
 *   Pusher.

 * */

@TeleOp(name="Test Pusher Only", group="Test")
public class TestPusherOnly extends LinearOpMode {
    private Runnerbot robot = new Runnerbot(this);

    static final double PUSHOUT_POS     =  1.00;     // Out front, to knock opponent Jewel off
    static final double RETRACT_POS     =  0.00;     // Tucked back to accept a Glyph.

// Define class members
    Servo   servo;
    double  position = RETRACT_POS; // Start at halfway position.
    @Override
    public void runOpMode() {
        // Initialization: map the pusher.
        // Connect to servo (Pusher)
        servo = hardwareMap.get(Servo.class, "Pusher");
				servo.setPosition(RETRACT_POS);
        telemetry.addData("Status",
          "Pusher ready for testing. ");
        telemetry.update();

        //Wait for testing to start.
        waitForStart();
        //elapsedTime.reset();
        telemetry.addData("Status", "RUNNING");
        telemetry.update();

        while(opModeIsActive()) {
            // Set the servo to the new position and pause;
            servo.setPosition(position);

            if (gamepad1.y) { // yellow button, used for moving pusher so it pushes the Glyph out.
                position = PUSHOUT_POS;
            }
            if (gamepad1.a) { // green button, used for moving pusher in, to allow loading Glyph.
                position = RETRACT_POS;
            } else {
                position = gamepad1.right_stick_y;
            }


            telemetry.addData("Joystick", gamepad1.right_stick_y);
            telemetry.addData(". Position", servo.getPosition());

            telemetry.update();
        }
    }
}
