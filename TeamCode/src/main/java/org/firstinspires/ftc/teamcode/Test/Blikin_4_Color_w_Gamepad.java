package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*
 * Displays 4 different patterns of a REV Robotics Blinkin LED Driver.
 * Use the gamepad buttons to switch between the different patterns.
 * Configure the driver on a servo port, and name it "blinkin".
 *
 * Displays the first pattern upon init.
 */
@TeleOp(name="Blinkin 4 Color with Gamepad")
//@Disabled
public class Blikin_4_Color_w_Gamepad extends OpMode {

     /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private final static int GAMEPAD_LOCKOUT = 500;

    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    Telemetry.Item patternName;
    Deadline gamepadRateLimit;

    RevBlinkinLedDriver.BlinkinPattern pattern1 = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    RevBlinkinLedDriver.BlinkinPattern pattern2 = RevBlinkinLedDriver.BlinkinPattern.SINELON_PARTY_PALETTE;
    RevBlinkinLedDriver.BlinkinPattern pattern3 = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE;
    RevBlinkinLedDriver.BlinkinPattern pattern4 = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE;

    @Override
    public void init()
    {

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin"); // assign to a servo port
        pattern = pattern1; // sets pattern to 1 when init is pressed
        blinkinLedDriver.setPattern(pattern); //method that actually sets the pattern
        patternName = telemetry.addData("Pattern: ", pattern.toString());
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
    }

    @Override
    public void loop()
    {
        handleGamepad(); // everything related to the LEDS is captured inside this one method


    }

    /*
     * handleGamepad
     *
     * Responds to a gamepad button press.  Demonstrates rate limiting for
     * button presses.  If loop() is called every 10ms and and you don't rate
     * limit, then any given button press may register as multiple button presses,
     * which in this application is problematic.
     *
    */
    protected void handleGamepad()
    {
        if (!gamepadRateLimit.hasExpired()) {
            return; // this will return back to the loop() if the 500 ms timer has NOT expired. This is so
            // is the driver bumps the button twice in a row, the pattern will not change until 500ms has passed.
            // this would be good for general teleop use in the future. Like for intake on and intake off with the same button.
        }

        if (gamepad1.a) {
            pattern = pattern1;
            displayPattern();
            gamepadRateLimit.reset();
        } else if (gamepad1.b) {
            pattern = pattern2;;
            displayPattern();
            gamepadRateLimit.reset();
        }
        else if (gamepad1.x) {
            pattern = pattern3;;
            displayPattern();
            gamepadRateLimit.reset();
        }
        else if (gamepad1.y) {
            pattern = pattern4;;
            displayPattern();
            gamepadRateLimit.reset();
        }



}

    private void displayPattern() {
        blinkinLedDriver.setPattern(pattern); // notice we are not passing the "pattern" with the method. Apparently since we are inside of the loop() we know what pattern is.
        patternName.setValue(pattern.toString());
    }
    }