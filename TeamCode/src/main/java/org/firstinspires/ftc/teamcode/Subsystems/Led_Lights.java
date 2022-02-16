package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

/*
 * Display patterns of a REV Robotics Blinkin LED Driver.
 * AUTO mode cycles through all of the patterns.
 * MANUAL mode allows the user to manually change patterns using the
 * left and right bumpers of a gamepad.
 *
 * Configure the driver on a servo port, and name it "blinkin".
 *
 * Displays the first pattern upon init.
 */
@TeleOp(name="LedLights")
//@Disabled
public class Led_Lights {


    /*
     * Change the pattern every 10 seconds in AUTO mode.
     */
    private final static int LED_PERIOD = 10;

    /*
     * Rate limit gamepad button presses to every 500ms.
     */
    private final static int GAMEPAD_LOCKOUT = 500;

    public RevBlinkinLedDriver blinkinLedDriver;

    public RevBlinkinLedDriver.BlinkinPattern pattern1 = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    public RevBlinkinLedDriver.BlinkinPattern pattern2 = RevBlinkinLedDriver.BlinkinPattern.SINELON_PARTY_PALETTE;
    public RevBlinkinLedDriver.BlinkinPattern pattern3 = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE;
    public RevBlinkinLedDriver.BlinkinPattern pattern4 = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE;


    Deadline ledCycleDeadline;
    Deadline gamepadRateLimit;

    public void init(HardwareMap hardwareMap)
    {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLedDriver.setPattern(pattern1);

        ledCycleDeadline = new Deadline(LED_PERIOD, TimeUnit.SECONDS);
        gamepadRateLimit = new Deadline(GAMEPAD_LOCKOUT, TimeUnit.MILLISECONDS);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern)
    {
        setPattern(pattern);
    }
}