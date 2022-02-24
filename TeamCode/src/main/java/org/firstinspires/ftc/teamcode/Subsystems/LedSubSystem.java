package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LedSubSystem {

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;
    //public RevBlinkinLedDriver.BlinkinPattern telemetryPattern;



    public RevBlinkinLedDriver.BlinkinPattern pattern1 = RevBlinkinLedDriver.BlinkinPattern.WHITE; // good for the 12V strip
    public RevBlinkinLedDriver.BlinkinPattern pattern2 = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
    public RevBlinkinLedDriver.BlinkinPattern pattern3 = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
    public RevBlinkinLedDriver.BlinkinPattern pattern4 = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
    public RevBlinkinLedDriver.BlinkinPattern telemetryPattern = RevBlinkinLedDriver.BlinkinPattern.WHITE; //initial setting

    public void init(HardwareMap hwMap) {

    blinkinLedDriver =hwMap.get(RevBlinkinLedDriver .class,"blinkin"); // assign to a servo port
    pattern = pattern1; // sets pattern to 1 when init is pressed
    blinkinLedDriver.setPattern(pattern); //method that actually sets the pattern


    }


    public void  displayPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinLedDriver.setPattern(pattern);


    }

}
