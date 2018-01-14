package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OpMode moves Paddle to one of two positions, MAX_POS and MIN_POS, respectively on game
 * pad buttons Y and A.
 * The code is structured as a LinearOpMode.
 *
 * This code assumes a Servo configured with the name "Paddle" as is found on Runnerbot.
 * v 0.1 Refactored 12/28/17 jmr from arm test opmode for Sliderbot. Tests only the minimum
 *   paddle.
 * Antonio Robles v 0.2 1/2/18 Changed the knock pos/retract position equation to test and see why
 *   it did not start at the halfway position.
 * Antonio Robles v 0.21 1/2/18 Added a joystick to the program to move the paddle in the y
 *   direction(up/down). He and his dad don't like the paddle position when joystick released. JMR
 *   doesn't either; release drives paddle to KNOCK_POS. Whole forward half of motion range is
 *   ignored; paddle stays in KNOCK_POS.
 * JMR v 0.22 planning for released joystick to drive paddle to RETRACT_POS, all the way
 *   forward to KNOCK_POS, forward joystick range from release position to cover whole movement
 *   range, all joystick backward range ignored.
 * */

@TeleOp(name="Test Paddle Only", group="Test")
public class TestPaddleOnlyMode extends LinearOpMode {
    private Runnerbot robot = new Runnerbot(this);

    static final double KNOCK_POS       =  1.00;     // Out front, to knock opponent Jewel off
    static final double RETRACT_POS     =  0.00;     // Folded back onto robot. Drive this way.

// Define class members
    Servo   servo;
    double  position = (KNOCK_POS + RETRACT_POS) / 2; // Start at halfway position. Sign was -
    @Override
    public void runOpMode() {
        // Initialization: map the paddle.
        // Connect to servo (Paddle)
        servo = hardwareMap.get(Servo.class, "Paddle");
				servo.setPosition(RETRACT_POS);
        telemetry.addData("Status",
          "Paddle initialized, waiting to test it. ");
        telemetry.update();

        //Wait for testing to start.
        waitForStart();
        //elapsedTime.reset();
        telemetry.addData("Status", "RUNNING");
        telemetry.update();

        while(opModeIsActive()) {
            // Set the servo to the new position and pause;
            servo.setPosition(position);

            if(gamepad1.y) { // yellow button, used for moving paddle to one pre-set position.
                position = KNOCK_POS;
            }
            if (gamepad1.a) { // green button, used for moving paddle to other pre-set position.
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
