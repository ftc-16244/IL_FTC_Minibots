package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.LedSubSystem;
/* This opmode will work with either the 12V or 5V LED strips. Some of the patterns listed in
the Blinkin Users Manual obviously look better with the 5V strips. The solid colors are a good
option for the 12V single color. Configuring the Blinkin as a SERVO will work but the colors
are not predictable. SImple change the names of Colors 1 - 4 in the LED subsystem as needed.d
More buttons can also be added if more than 4 colres are desired.
 */

@TeleOp(name="Linear LED", group="Teleop")

public class Linear_LED extends LinearOpMode{

    LedSubSystem leds = new LedSubSystem(); // instantiate an istance of LED Lights here


    @Override
    public void runOpMode() {

        leds.init(hardwareMap);
        telemetry.addData("Pattern: ", leds.pattern.toString());
        telemetry.update();

        waitForStart();

            while (!isStopRequested()) {

                if (gamepad1.dpad_up) {
                    leds.displayPattern(leds.pattern1);
                    telemetry.addData("Pattern: ",  leds.pattern1.toString());

                } else if (gamepad1.dpad_left) {
                    leds.displayPattern(leds.pattern2);
                    telemetry.addData("Pattern: ", leds.pattern2.toString());


                } else if (gamepad1.dpad_down) {
                    leds.displayPattern(leds.pattern3);
                    telemetry.addData("Pattern: ", leds.pattern3.toString());

                } else if (gamepad1.dpad_right) {
                    leds.displayPattern(leds.pattern4);
                    telemetry.addData("Pattern: ", leds.pattern4.toString());

                }

            telemetry.update();
            }
        }

    }
