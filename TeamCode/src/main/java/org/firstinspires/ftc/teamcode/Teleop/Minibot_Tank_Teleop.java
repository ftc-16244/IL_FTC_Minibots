package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Autonomous.BasicMiniBotMeccanum;
import org.firstinspires.ftc.teamcode.Autonomous.BasicMiniBotTank;
import org.firstinspires.ftc.teamcode.Subsystems.Four_Motor_Minibot_Meccanum_Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Two_Motor_Minibot_Tank_Drivetrain;

@TeleOp(name="Tank Drive MiniBot Teleop", group="Teleop")
@Disabled

public class Minibot_Tank_Teleop extends BasicMiniBotTank {

    // Motors not declared here becase they are part of BasicMiniBotTank which is extended
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {

        drivetrain.init(hardwareMap, false);



        waitForStart();
        /////////////////////////////////////////////////////////////////////////////////////////


        runtime.reset();
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
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;


            // Send calculated power to wheels
            drivetrain.leftFront.setPower(leftPower);
            drivetrain.rightFront.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

            if (gamepad1.x){
                sideServo.moveServoLeft();
            }
            if (gamepad1.b){
                sideServo.moveServoRight();
            }
        }

    }
}

