/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Version History
 * v 0.1 Josh Maros. Ran well in lab. A similar one of his for the Sliderbot ran well at Meet 2.
 * v 0.11 JMR added this version history.
 *
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Runnerbot Just Drive", group="Linear Opmode")
public class RunnerBotJustDrive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private long nudgeStart = 0; //Records the time a user first pressed a
    // button to start a nudge.
    private static final long NUDGE_TIME = 200; //How many milliseconds a
    // nudge should be run for.

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            //Squaring inputs allows for more precise control over small movements while still
            //retaining the maximum possible speed.
            /*  Through testing, I found this only works well on large joysticks (the kind FRC uses)
            if(drive > 0)
                drive = drive * drive;
            else
                drive = -drive * drive;
            if(turn > 0)
                turn = turn * turn;
            else
                turn = -turn * turn;
            */

            //The driver can press a dpad button to nudge the robot in a particular direction.
            boolean nudgePressed = gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left ||
                    gamepad1.dpad_right;
            if(nudgePressed) {
                drive = 0.0;
                turn = 0.0;
                //nudgeTimer is used to perform a nudge for a specified number of frames, even if
                //the user continues to hold the nudge button.
                if(nudgeStart == 0) {
                    nudgeStart = System.currentTimeMillis();
                }
                if(nudgeStart + NUDGE_TIME > System.currentTimeMillis()) {
                    if(gamepad1.dpad_up) {
                        drive = 0.5;
                    } else if(gamepad1.dpad_down) {
                        drive = -0.5;
                    } else if(gamepad1.dpad_left) {
                        turn = -0.5;
                    } else if(gamepad1.dpad_right) {
                        turn = 0.5;
                    }
                }
            } else {
                nudgeStart = 0;
            }

            //Reduce drive speed if left trigger or bumper are pressed.
            if(gamepad1.left_bumper || (gamepad1.left_trigger > 0.5)) {
                drive *= 0.5;
            }
            //Reduce turn speed if right trigger or bumper are pressed.
            if(gamepad1.right_bumper || (gamepad1.right_trigger > 0.5)) {
                turn *= 0.5;
            }

            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
