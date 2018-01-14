package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by josh on 10/31/17: drive a Sliderbot with this
 */

/*
 * Bot structure:
 *y       x
 * fl---fr
 *  |\ /|
 *  |/ \|
 * bl---br
 *-x     -y
 */

@TeleOp(name="Mecanum Wheel Teleop", group="Linear Opmode")
public class MecanumOpMode extends LinearOpMode {
    private ElapsedTime elapsedTime = new ElapsedTime();
    private DcMotor frontRight, backLeft, backRight;
    private DcMotor frontLeft = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized, waiting for match to start.");
        telemetry.update();

        //Load all the motors.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Set their turn directions. Coaxial facing motors must be set
        // opposite to both turn wheels the same way.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //Wait for the match to start.
        waitForStart();
        elapsedTime.reset();

        while(opModeIsActive()) {
            double x = gamepad1.right_stick_x, y = gamepad1.right_stick_y;
            //Squaring input values makes fine movements easier, while the max speed stays the same.
            x = x * x;
            y = y * y;
            double magnitude = Math.sqrt(x * x + y * y); //a^2+b^2=c^2
            double angle = 0.0;
            if(x != 0.0) { //To prevent DIV0 errors.
                angle = Math.atan(y / x); //tan(theta) = opposite (y) / adjacent (x)
                if(y < 0.0) {
                    angle += 180.0; //Because 1 / 1 and -1 / -1 give the same angle.
                }
            }
            angle -= 45.0; //Refer to diagram at top of source. The wheels' coordinate system is
                           //45 degrees off from the robot's, making this correction necessary.
            double wheelX = Math.cos(angle), wheelY = Math.sin(angle);
            //'Normalization'. This gives a competitive advantage. For example, when going at 45
            //degrees, plugging that into sin and cos gives sqrt(2) for both axes. But the robot
            //can go faster by applying max power to both axes. This still preserves the direction,
            //because the ratio of the speeds of each axis is still the same. This code does a
            //similar process, but works for all directions.
            double wheelMax = Math.max(Math.abs(wheelX), Math.abs(wheelY));
            wheelX /= wheelMax;
            wheelY /= wheelMax;
            //And then scale those normalized values based on how fast the driver wants it to go.
            wheelX *= magnitude;
            wheelY *= magnitude;
            //Refer to diagram for explanation of this.
            frontRight.setPower(wheelX);
            backLeft.setPower(wheelX);
            frontLeft.setPower(wheelY);
            backRight.setPower(wheelY);

            telemetry.addData("Status", "Running (" + elapsedTime.toString() + ")");
            telemetry.addData("Wheel X", "%.2f", wheelX);
            telemetry.addData("Wheel Y", "%.2f", wheelY);
            telemetry.update();
        }
    }
}
