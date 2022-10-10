package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Autonomous.BasicMiniBotMeccanum;


@TeleOp(name="Mecanum MiniBot Teleop", group="Teleop")
//@Disabled

public class Minibot_Mecanum_Teleop extends BasicMiniBotMeccanum {
    //Led_Lights led_lights = new Led_Lights();
    @Override
    public void runOpMode() {

        drivetrain.init(hardwareMap, true);
        sideServo.init(hardwareMap); // keep track of capital S and lower case s on this one. It is confusing

        sideServo.moveServoCenter(); // puts servo in the center to simulate starting a match


        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double lf, lr, rf, rr;

            lf = (y + x + rx); // forward + turn right + strafe right
            lr = (y - x + rx);
            rf = (y - x - rx);
            rr = (y + x - rx);

            if (Math.abs(lf) > 1 || Math.abs(lr) > 1 || Math.abs(rf) > 1 || Math.abs(rr) > 1) {
                double max = 0;
                max = Math.max(Math.abs(lf), Math.abs(lr));
                max = Math.max(Math.abs(rf), max);
                max = Math.max(Math.abs(rr), max);

                // scales output if y + x + rx >1
                lf /= max;
                lr /= max;
                rf /= max;
                rr /= max;

            }


            drivetrain.leftFront.setPower(lf);
            drivetrain.leftRear.setPower(lr);
            drivetrain.rightFront.setPower(rf);
            drivetrain.rightRear.setPower(rr);

            telemetry.addData("Left Stick Y-Fwd", "%5.2f", y);
            telemetry.addData("Right  Stick X-Turn", "%5.2f", x);
            telemetry.addData("Left Stick RX-Rotation", "%5.2f", rx);
            telemetry.update();

            // Implement Functions on Gamepad #1

            if (gamepad1.x) {
                sideServo.moveServoLeft();
                // let the servo move
                sleep(500);
            }
            if (gamepad1.b) {
                sideServo.moveServoRight();
                sleep(500);
            }

        }
    }

}