package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomous.BasicMiniBotMeccanum;
import org.firstinspires.ftc.teamcode.Subsystems.LedSubSystem;

@TeleOp(name="Mecanum MiniBot Teleop with LED", group="Teleop")

public class Minibot_Mecanum_Teleop_w_LED extends BasicMiniBotMeccanum {

    LedSubSystem leds = new LedSubSystem(); // instantiate an istance of LED Lights here


    @Override
    public void runOpMode() {

       // Call subsystem init methods
        drivetrain.init(hardwareMap, true);
        leds.init(hardwareMap);
        sideServo.init(hardwareMap); // keep track of capital S and lower case s on this one. It is confusing

        // Set everything to its start position as required
        sideServo.moveServoCenter(); // puts servo in the center to simulate starting a match

        // Add telemetry
        telemetry.addData("Pattern: ", leds.pattern.toString()); // initial pattern is subsystem init method.
        telemetry.update();

        //////////////////////////////////////////WAIT FOR START////////////////////////////////////

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

            if (gamepad1.dpad_up) {
                leds.displayPattern(leds.pattern1);
                leds.telemetryPattern = leds.pattern1;


            } else if (gamepad1.dpad_left) {
                leds.displayPattern(leds.pattern2);
                leds.telemetryPattern = leds.pattern2;


            } else if (gamepad1.dpad_down) {
                leds.displayPattern(leds.pattern3);
                leds.telemetryPattern = leds.pattern3;

            } else if (gamepad1.dpad_right) {
                leds.displayPattern(leds.pattern4);
                leds.telemetryPattern = leds.pattern4;

        }
            telemetry.addData("Pattern: ",  leds.telemetryPattern.toString());
            telemetry.addData("Left Stick Y-Fwd", "%5.2f", y);
            telemetry.addData("Right  Stick X-Turn", "%5.2f", x);
            telemetry.addData("Left Stick RX-Rotation", "%5.2f", rx);
            telemetry.update();
        }
    }

}