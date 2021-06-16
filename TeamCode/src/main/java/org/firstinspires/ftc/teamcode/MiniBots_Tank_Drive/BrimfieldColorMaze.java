package org.firstinspires.ftc.teamcode.MiniBots_Tank_Drive;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain_v3;


@Autonomous(name="Brimfield Color Maze", group="Au@to")
//@Disabled

public class BrimfieldColorMaze  extends BasicMiniBotTank {
    int turnIndicator = 0;
    final float[] hsvValues = new float[3];
    float hue = 150;
    float saturation = 80;
    float BlueMinHue = 165;
    float BlueMaxHue = 225;
    float RedMinHue = 20;
    float RedMaxHue = 350;

    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap);
        // Gyro set-up
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Init gyro parameters then calibrate
        drivetrain.imu.initialize(parameters);

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        // Encoder rest is handled in the Drivetrain init in Drivetrain class

        // Calibrate gyro

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && !drivetrain.imu.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", drivetrain.imu.getCalibrationStatus().toString());
        /** Wait for the game to begin */


        telemetry.update();

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        ////////////////////////////////////////////////////////////////////////////////////////////
        while (!isStopRequested()) {
            // update these constantly from the color sensor
           hue          =   hsvValues[0];
           saturation   =   hsvValues[1];




        }



    }


    // Unique method for color maze special case

    public void gyroDrive4Maze ( double speed,
                            double distance,
                            double angle, double timeout, int turnIndicator) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        totalError = 0;
        lasterror = 0;
        telemetry.addData("gyroDrive Activated", "Complete");
        // Ensure that the opmode is still active
        // Use timeout in case robot gets stuck in mid path.
        // Also a way to keep integral term from winding up to bad.
        if (opModeIsActive() & drivetime.time() < timeout) {

            // Determine new target position in ticks/ counts then pass to motor controller
            moveCounts = (int)(distance * Drivetrain_v3.COUNTS_PER_INCH);
            newLeftTarget = drivetrain.leftFront.getCurrentPosition() + moveCounts;
            newRightTarget = drivetrain.rightFront.getCurrentPosition() + moveCounts;

            // Set Target using the calculated umber of ticks/counts

            drivetrain.leftFront.setTargetPosition(newLeftTarget);
            drivetrain.rightFront.setTargetPosition(newRightTarget);
            // Tell motor control to use encoders to go to target tick count.

            drivetrain.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drivetrain.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            // Up to now this is all the same as a drive by encoder opmode.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            drivetrain.leftFront.setPower(speed);
            drivetrain.rightFront.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            // once one motor gets to the target number of ticks it is no longer "busy"
            // and isbusy in false causing the loop to end.
            while (opModeIsActive() &&
                    (drivetrain.leftFront.isBusy() && drivetrain.rightFront.isBusy()) && turnIndicator == 0) {

                // adjust relative speed based on heading error.
                // Positive angle means drifting to the left so need to steer to the
                // right to get back on track.
                error = getError(angle);
                steer = getSteer(error, Kp_DRIVE, Ki_DRIVE, Kd_DRIVE);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                drivetrain.leftFront.setPower(leftSpeed);
                drivetrain.rightFront.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      drivetrain.leftFront.getCurrentPosition(),
                        drivetrain.rightFront.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();


            }

            // Stop all motion;
            drivetrain.leftFront.setPower(0);
            drivetrain.rightFront.setPower(0);

            // Turn off RUN_TO_POSITION
            drivetrain.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            drivetrain.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        drivetime.reset(); // reset the timer for the next function call
    }

}
