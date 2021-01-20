package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Enums.ShooterState;
import org.firstinspires.ftc.teamcode.Enums.WobbleTargetZone;

import java.util.List;

@Autonomous(name="EXP_Meet 2 Auto", group="Test")
//@Disabled // Leave disabled until ready to test

// This opmode EXTENDS BasicAutonomous and actually does the same thing as BasicAutonomous
// The goal here was to extend a base class with all the methods and prove it works just the same.

// Place robot on the right most blue line when facing the goal. Robot should be placed such that
// as it drives straight ahead it will not hit the stack of rings. The left edge of teh left most intake wheel
// needs to line up with the right edge of the tape on the floor. This is so the new wobble gripper and wobble
// goal will not hit the 4 stack of rings on the first part of auto.


// Alignment Position RH Blue Line when facing the goal
//    X     B       X       X
//    X     B       X       X
//    X     B       X       X
//    X     B       X       X
//    X     B       X       X

public class EXP_Meet_2_Auto extends BasicAutonomous {
    private static  final double        extraRingShootTimeAllowed   = 2; //  timer for shooting the single ring unique to this opMode
    private static  final double        autoShootTimeAllowed        = 4;//
    private static final  double        autoRingCollectTimeAllowed  = 0.6; // time allowed to let the single ring to get picked up
    private static final double         autoRingCollectTimeAllowed_C = 2;
    private static final double         shooterStartUpTimeAllowed   = 1.25;
    public static final double          DRIVE_SPEED                 = 0.70;     // Nominal speed for better accuracy.

    @Override
    public void runOpMode() {

        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();

            // See BasicAutonomous for details on camera zoom settings.
            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            tfod.setZoom(2.5, 1.78);
        }

        // Call init methods in the various subsystems
        // if "null exception" occurs it is probably because the hardware init is not called below.
        drivetrain.init(hardwareMap);
        wobble.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        elevator.init(hardwareMap);
        m_Ring_Spreader.init(hardwareMap);

        // move implements to start position. Note, 18x18x18 inch cube has to be maintained
        // until start is pressed. Position servos and motors here so human error and set-up is not
        // as critical. Team needs to focus on robot alignment to the field.

        shooter.shooterReload(); // reload = flipper back, stacker mostly down, shooter off
        // nothing here for wobble goal yet. Gravity will take care of most of it.
        // the wobble gripper is automatically opened during the wobble init.

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
            sleep(100);
            idle();
        }

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", drivetrain.imu.getCalibrationStatus().toString());
        /** Wait for the game to begin */
        telemetry.addData("Square", Square);

        telemetry.update();

        /////////////////////////////////////////////////////////////////////////////////////////////
        waitForStart();
        ////////////////////////////////////////////////////////////////////////////////////////////
        tfTime.reset(); //  reset the TF timer
        while (tfTime.time() < tfSenseTime && opModeIsActive()) { // need to let TF find the target so timer runs to let it do this
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        ///
                        StackSize = recognition.getLabel();
                        //telemetry.addData("Target", Target);
                        if (StackSize == "Quad") {
                            Square = WobbleTargetZone.BLUE_C;
                            telemetry.addData("Square", Square);
                        } else if (StackSize == "Single") {
                            Square = WobbleTargetZone.BLUE_B;
                            telemetry.addData("Square", Square);

                        }

                    }
                    telemetry.update();
                }
            }
            if (tfod != null) {
                tfod.shutdown();
            }
        }
        // Pick up the Wobble Goal before moving.
        // Sleep statements help let things settle before moving on.
        wobble.GripperPartOpen();
        //wobble.ArmExtend();
        sleep(250);
        wobble.wobbleWristDown();
        sleep(250);
        wobble.GripperClose();
        //sleep(250);
        //wobble.ArmCarryWobble();


        // After picking up the wobble goal the robot always goes to the same spot to shoot the 3 preloaded rings.
        // After delivering the rings, the switch case has the appropriate drive path to the identified Target Zone.

        drivetime.reset(); // reset because time starts when TF starts and time is up before we can call gyroDrive
        // Drive paths are initially all the same to get to the shooter location
        //gyroDrive(DRIVE_SPEED, 54.0, 0.0, 10);

        gyroDrive(DRIVE_SPEED, 54.0, 0.0, 10);// 54 total
        //gyroDrive(DRIVE_SPEED*.6, 10.0, 0.0, 10);// 54 total
        gyroTurn(TURN_SPEED,10,3); //Need to change this angle for the right line start point
        mShooterState = ShooterState.STATE_SHOOTER_ACTIVE;
        shooterStartUp(mShooterState, shooterStartUpTimeAllowed);
        //shoot3Rings(mShooterState, autoShootTimeAllowed);   // call method to start shooter and launch 3 rings. pass shooter state in case it is needed
        try {
            shooter.shoot_N_rings(3);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        shooter.shooterReload();
        drivetime.reset(); // reset because time starts when TF starts and time is up before we can call gyroDrive

        // Switch manages the 3 different Target Zone objectives based on the number of rings stacked up
        // Ring stack is none, one or 4 rings tall and is determined by a randomization process.
        // Robot has to read the stack height and set the Target Zone square state based on Vuforia/ Tensor Flow detection
        switch(Square){
            case BLUE_A: // no rings. 3 tiles (24 inches per tile) forward and 2 tiles to the left from start
                telemetry.addData("Going to BLUE A", "Target Zone");
                gyroTurn(TURN_SPEED,45,2);
                gyroTurn(TURN_SPEED*.4,60,3);
                gyroDrive(DRIVE_SPEED,27,60,4);
                sleep(500);
                wobble.GripperOpen();
                sleep(500);
                //wobble.ArmExtend();


                drivetime.reset();
                gyroDrive(DRIVE_SPEED*.5,-4,60,4);
                wobble.lowerWobbleClamp(); // back up so clamp doest catch on first wobble
                gyroTurn(TURN_SPEED, 155,4);
                gyroTurn(TURN_SPEED*.4, 180,3);

                gyroDrive(DRIVE_SPEED*.7,36,180,4);


                gyroTurn(TURN_SPEED*.4,157,3); 
                gyroDrive(DRIVE_SPEED*5,6.5,157,3);


                //wobble.ArmExtend();

                //sleep(500);
                wobble.GripperClose();
                sleep(1000);
                //wobble.liftPartial();
                drivetime.reset();
                gyroDrive(DRIVE_SPEED,-55,153,2); // backup with 2nd wobble goal
                gyroTurn(TURN_SPEED*.5,90,3);
                gyroDrive(DRIVE_SPEED,13,90,2);
               //wobble.GripperSuperOpen();
                wobble.GripperOpen();
                sleep(250);
                wobble.raiseWobbleClamp();
                sleep(250);
                gyroDrive(DRIVE_SPEED,-10,90,2);
                //wobble.ArmContract();
                wobble.wobbleWristStart();
                wobble.GripperClose();
                sleep(500);// Keeps power to servo on long enough to allow servo to move


                break;
            case BLUE_B: // one ring  4 tiles straight ahead
                // Drop off wobble
                telemetry.addData("Going to BLUE B", "Target Zone");
                gyroDrive(DRIVE_SPEED, 24.0, 10.0, 5);
                sleep(500);
                wobble.GripperOpen();
                sleep(500);

                // Go towards the single ring and get second wobble
                drivetime.reset();
                gyroDrive(DRIVE_SPEED,-4,10,3);

                drivetime.reset();
                gyroTurn(TURN_SPEED*0.75,150,4);
                gyroTurn(TURN_SPEED*0.4,167,3);
                m_Ring_Spreader.ringSpreaderDown(); // drop ring spreader arm to prevent jamming in case TF reads C and goes to B
                gyroDrive(DRIVE_SPEED*.7, 28, 167, 5);
                gyroDriveandCollectRings(DRIVE_SPEED,8,167,2); // was 8

                intake.Intakeon();
                elevator.ElevatorSpeedfast();
                // Go for second wobbble
                gyroTurn(TURN_SPEED,155,3);
                gyroTurn(TURN_SPEED*.45,150,2);
                wobble.lowerWobbleClamp();
                gyroDrive(DRIVE_SPEED,17,150,3);
                gyroDrive(DRIVE_SPEED*.45,7,148,3);
                wobble.GripperClose();
                sleep(500);
                gyroDrive(DRIVE_SPEED,-19,148,3);

                // Turn back to face the goal and shoot
                gyroTurn(TURN_SPEED*.8,25,3); //turn fast most of the way
                gyroTurn(TURN_SPEED*.4,-2,3);// turn slow to be accurate. Need to une PIDSs better instead

                gyroDrive(DRIVE_SPEED*.7, 7, -2, 3);

                intake.Intakeoff();
                elevator.Elevatoroff();

                //wobble.wobbleWristStart();
                //wobble.GripperClose();
                mShooterState = ShooterState.STATE_SHOOTER_ACTIVE;
                shooterStartUp(mShooterState, shooterStartUpTimeAllowed);

                try {
                    shooter.shoot_N_rings(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
               }

                drivetime.reset();
                gyroDrive(DRIVE_SPEED,21,-6,3);


                wobble.GripperOpen();
                sleep(250);
                wobble.wobbleWristStart();
                sleep(250);
                gyroDrive(DRIVE_SPEED,-3,-8,3);
                wobble.raiseWobbleClamp();
                m_Ring_Spreader.ringSpreaderUp();
                sleep(250);

                break;
            case BLUE_C: // four rings. 5 tiles forward and one tile to the left.
                // Drop off first wobble after shooting 3 rings
                telemetry.addData("Going to BLUE C", "Target Zone");
                gyroTurn(TURN_SPEED *.5,22,2);
                gyroDrive(DRIVE_SPEED, 54, 22,3);
                wobble.GripperOpen();
                sleep(500);

                // Move to the stack of 4 rings
                drivetime.reset();
                gyroDrive(DRIVE_SPEED, -25, 20,3);
                gyroTurn(TURN_SPEED,120,2);
                gyroTurn(TURN_SPEED*.5,180,2);
                m_Ring_Spreader.ringSpreaderDown(); // drop ring spreader arm to prevent jamming
                gyroDrive(DRIVE_SPEED,31,180,2); // last forward move before collecting
                drivetime.reset();

                // Collect from the stack of 4 rings
                gyroDriveandCollectRings(DRIVE_SPEED*.2,10,180,10); // collect from stack
                gyroDriveandCollectRings(DRIVE_SPEED*.6,-3,180,10); // back up to prevent jamming
                gyroDriveandCollectRings(DRIVE_SPEED*.2,7,180,10); // collect from stack
                gyroDriveandCollectRings(DRIVE_SPEED*.8,-14,180,10); // backup leave intake on

                // Keep intake and elevator running to get ring settled
                intake.Intakeon();
                elevator.ElevatorSpeedfast();
                wobble.wobbleWristStart(); // lift up wrist so it is not across the line
                wobble.GripperClose(); // close gripper so rings exiting shooter do not hit the grabber arm

                // Turn to face the goal and shoot rings from the stack
                drivetime.reset();
                gyroTurn(TURN_SPEED,40,2);
                gyroTurn(TURN_SPEED*.30,0 ,2);
                intake.Intakeoff();
                elevator.Elevatoroff();
                mShooterState = ShooterState.STATE_SHOOTER_ACTIVE;
                shooterStartUp(mShooterState, shooterStartUpTimeAllowed);
                try {
                    shooter.shoot_N_rings(2);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                drivetime.reset();

                // Pull ahead to the line
                gyroDrive(DRIVE_SPEED, 10, 0,3);
                m_Ring_Spreader.ringSpreaderUp();
                sleep(500);
                break;
        }


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }






}
