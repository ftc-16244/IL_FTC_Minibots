package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Enums.DriveSpeedState;
import org.firstinspires.ftc.teamcode.Enums.LiftMode;
import org.firstinspires.ftc.teamcode.Enums.RingCollectionState;
import org.firstinspires.ftc.teamcode.Enums.WobbleLiftPosn;
import org.firstinspires.ftc.teamcode.Enums.WristPosn;
import org.firstinspires.ftc.teamcode.Subsystems.Debouce;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain_v3;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Ring_Spreader;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter_VelCtrl;
import org.firstinspires.ftc.teamcode.Subsystems.Wobblegoal;

import java.util.Base64;

@TeleOp(name="Meet 4 Teleop with Manual Lift", group="Teleop")
//@Disabled
public class Meet_4_Teleop_w_Manual_Lift_Option extends OpMode {


    /* Declare OpMode members. */

    private ElapsedTime runtime     = new ElapsedTime();
    public Drivetrain_v3        drivetrain  = new Drivetrain_v3(true);   // Use subsystem Drivetrain
    public Shooter_VelCtrl shooter     = new Shooter_VelCtrl(); //experiment to see if thsi helps
    public Intake               intake      = new Intake();
    public Wobblegoal           wobble  = new Wobblegoal();
    public Elevator elevator    = new Elevator();
    public Ring_Spreader m_Ring_Spreader = new Ring_Spreader();

    public ElapsedTime gripperCloseTimer = new ElapsedTime();
    //public ElapsedTime debounceTimer = new ElapsedTime();
    private Debouce mdebounce = new Debouce();

    // ENUMS
    private LiftMode liftMode=LiftMode.ENCODER;
    private DriveSpeedState  currDriveState;
    private RingCollectionState ringCollectorState;
    private double gripperCloseTime = 1.0;

    public WobbleLiftPosn liftposn =WobbleLiftPosn.IDLE; // Default target zone
    public WristPosn wristPosn = WristPosn.PARK;

    private ElapsedTime wristTimer = new ElapsedTime();
    private double wristParkDelay = 3;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
   public void init() {

        double wristRestTime = 3;
        ElapsedTime wristResetTimer = new ElapsedTime();
        /* Initialize the hardware variables.
        * The init() method of the hardware class does all the work here
        */
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        wobble.init(hardwareMap);
        elevator.init(hardwareMap);
        shooter.init(hardwareMap);
        m_Ring_Spreader.init(hardwareMap);
        //newState(currDriveState);
        currDriveState = DriveSpeedState.DRIVE_FAST; // initialize robot to FAST
        ringCollectorState = RingCollectionState.OFF;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");

        telemetry.addData("FAST DRIVE","Mode");//
        //telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double speedfactor = 0.5;
        double manualSpeed = 0;

        // Ultimate Goal
        //Gamepad 1
        //  Drivetrain (left/right sticks)
        //  Intake on
        //  Intake off
        //  Intake reverse
        //  Wobble (at least 4 function)

        // Gamepad 2
        // Shooter slow (mid goal)
        // Shooter fast (high goal)
        // Shooter reverse
        // stacker uo
        // stacker down
        // flipper in
        // flipper out


        //========================================
        // GAME PAD 1
        //========================================
        // left joystick is assigned to drive speed
        drive = -gamepad1.left_stick_y;
        // right joystick is for turning
        turn = gamepad1.right_stick_x;
        // Combine drive and turn for blended motion.
        left = drive + turn;
        right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0) {
            left /= max; // does this to stay within the limit and keeps the ratio the same
            right /= max;
        }

        // Gamepad 1 Buttons


        if (gamepad1.left_bumper && ringCollectorState == RingCollectionState.OFF) {
            shooter.flipperBackward();
            shooter.stackerMoveToMidLoad();
            ringCollectorState = RingCollectionState.COLLECT;
            telemetry.addData("Collector State", ringCollectorState);
            mdebounce.debounce(175); // need to pause for a few ms to let drive release the button

        }
        if (gamepad1.left_bumper && ringCollectorState == RingCollectionState.COLLECT) {
            shooter.flipperBackward();
            shooter.stackerMoveToMidLoad();
            ringCollectorState = RingCollectionState.OFF;
            telemetry.addData("Collector State", ringCollectorState);
            mdebounce.debounce(175);
        }


        if (gamepad1.right_bumper && ringCollectorState == RingCollectionState.OFF) {
            shooter.flipperBackward();
            shooter.stackerMoveToReload();
            ringCollectorState = RingCollectionState.EJECT;
            telemetry.addData("Collector State", ringCollectorState);
            mdebounce.debounce(175);

        }

        if (gamepad1.right_bumper && ringCollectorState == RingCollectionState.EJECT) {
            shooter.flipperBackward();
            shooter.stackerMoveToReload();
            ringCollectorState = RingCollectionState.OFF;
            telemetry.addData("Collector State", ringCollectorState);
            mdebounce.debounce(175);

        }

        if (gamepad1.x) {
            //shooter.shooterReload();
            shooter.stackerMoveToReload();
            telemetry.addData("Stacker Reset", "Complete ");

        }
        if (gamepad1.y) {
            shooter.shootOneRingHigh();
            //shooter.shootMiddleGoal();
            ringCollectorState = RingCollectionState.OFF;

            telemetry.addData("Shooter High", "Complete ");
        }

        if (gamepad1.a) {
            shooter.shooterReload();
            //shooter.shooterOff();
            telemetry.addData("Shooter High", "Complete ");
        }
        if (gamepad1.b) {
            shooter.stackerMoveToShoot();
            ringCollectorState = RingCollectionState.OFF;
            telemetry.addData("Stacker Ready to Shoot", "Complete ");
        }
        if (gamepad1.left_trigger > 0.25) {
            shooter.flipperForward();
            debounce(600);
            telemetry.addData("Flipper Fwd", "Complete ");
            shooter.flipperBackward();
            debounce(600);
        }
        if (gamepad1.right_trigger > 0.25) {
            //shooter.flipperBackward();
            //telemetry.addData("Flipper Back", "Complete ");
            shooter.shootonePowerShots();
            currDriveState = DriveSpeedState.DRIVE_SLOW;
            telemetry.addData("SHooter Low for Power Shots", "Complete ");
        }

        // Gamepad 1 Bumpers - for Speed Control
        // set-up drive speed states on bumpers
        if (gamepad1.left_stick_button) {
            currDriveState = DriveSpeedState.DRIVE_FAST;
        }
        if (gamepad1.right_stick_button) {
            currDriveState = DriveSpeedState.DRIVE_SLOW;
        }


        // Wobble Controls

        if (gamepad1.dpad_left) {
            wobble.GripperOpen();
            //wobble.ArmExtend();
            wristPosn = WristPosn.DOWN;
            wobble.lowerWobbleClamp();
            //wobble.wobbleWristDown();

            telemetry.addData("Ready to rab Wobble", "Complete ");
        }

        if (gamepad1.dpad_up) {
            gripperCloseTimer.reset();
            wobble.GripperClose();
            //while (gripperCloseTimer.time() < gripperCloseTime){

            // stall program so gripper can close
            // not necessary in Linear Opmode just in iterative
            //more than a couple seconds and this will trow error
            //}

            wobble.raiseWobbleClamp();
            //wobble.readyToGrabGoal();
            telemetry.addData("Carrying Wobble", "Complete ");
        }
        if (gamepad1.dpad_right) {
            wobble.GripperOpen();


            telemetry.addData("Dropping Wobble", "Complete ");
        }
        if (gamepad1.dpad_down) {

            wobble.GripperClose();
            //wobble.LiftLower();
            liftposn = WobbleLiftPosn.DOWN; // lift position state
            //wobble.wobbleWristUp();
            wristPosn = WristPosn.UP; // wrist position state
            wobble.raiseWobbleClamp();

        }
        // Hold dpad down then move right hand (turn stick) back (in the Y direction) to park wrist
        if (gamepad1.dpad_down && gamepad1.right_stick_y > 0.5) {

            wristPosn = WristPosn.PARK; //
        }


        if (gamepad1.back) {

            //wobble.LiftRise();
            liftposn = WobbleLiftPosn.UP;
            //wobble.raiseWobbleClamp();
            wristPosn = WristPosn.DOWN; // wrist position state

        }

        //========================================
        // GAME PAD 2 Mainly Wobble
        //========================================


        if (gamepad2.x) {
            m_Ring_Spreader.ringSpreaderUp();
        }
        if (gamepad2.b) {
            m_Ring_Spreader.ringSpreaderDown();
        }

        if (gamepad2.back) {
            shooter.stackerMoveToDump();
            intake.Intakeoff();
            elevator.Elevatoroff();
        }
        manualSpeed = -gamepad2.left_stick_y;
        if (gamepad2.left_bumper) {
            liftMode = LiftMode.MANUAL;
            wobble.WobbleLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (gamepad2.right_bumper) {
            liftMode = LiftMode.ENCODER;
            wobble.WobbleLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wobble.WobbleLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (gamepad2.dpad_left && liftMode == liftMode.MANUAL) {
            wobble.GripperOpen();
            wristPosn = wristPosn.DOWN;

        }
       // switch case to determine what mode the arm needs to operate in.


        // switch case for the drive speed state

        switch(currDriveState) {

            case DRIVE_FAST:
                telemetry.addData("Drive Speed",currDriveState);
                drivetrain.leftFront.setPower(left);
                drivetrain.rightFront.setPower(right);
                //leftFront.setPower(left);
                //rightFront.setPower(right);

                // Send telemetry message to signify robot running;
                telemetry.addData("left",  "%.2f", left);
                telemetry.addData("right", "%.2f", right);
                break;

            case DRIVE_SLOW:
                telemetry.addData("Drive Speed",currDriveState);
                drivetrain.leftFront.setPower(left*speedfactor);
                drivetrain.rightFront.setPower(right*speedfactor);
                //leftFront.setPower(left*speedfactor);
                //rightFront.setPower(right*speedfactor);

                // Send telemetry message to signify robot running;
                telemetry.addData("left",  "%.2f", left);
                telemetry.addData("right", "%.2f", right);
                break;
        }

        // State machine for the wobble lift
        if (liftMode==LiftMode.ENCODER){
            telemetry.addData("liftMode", liftMode);
            switch (liftposn) {

                case DOWN:
                    telemetry.addData("Lift Position", liftposn);
                    wobble.LiftLower();
                    if (wobble.getLiftHeight() < 0.15) {
                        liftposn = WobbleLiftPosn.IDLE;
                        wobble.WobbleLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    break;

                case UP:
                    telemetry.addData("Lift Position", liftposn);
                    wobble.LiftRise();
                    //wobble.wobbleWristDown();


                    break;

                case IDLE:
                    telemetry.addData("Lift Position", liftposn);
                    wobble.LiftIdle();


                    break;
            }
        }
        if (liftMode== liftMode.MANUAL){
            telemetry.addData("Lift Position", liftposn);
            wobble.WobbleLift.setPower(manualSpeed);
        }

        // Wrist Position

        switch(wristPosn) {

            case PARK:
                telemetry.addData("Wrist  Position",wristPosn);
                wobble.wobbleWristStart();

                break;

            case UP:
                telemetry.addData("Wrist  Position",wristPosn);
                wobble.wobbleWristUp();

                break;

            case DOWN:
                telemetry.addData("Wrist  Position",wristPosn);
                wobble.wobbleWristDown();


                break;
        }


        // States for intake direction
        switch(ringCollectorState) {

            case OFF:
                telemetry.addData("Collector State",ringCollectorState);
                intake.Intakeoff();;
                elevator.Elevatoroff();

                break;

            case COLLECT:
                telemetry.addData("Collector State",ringCollectorState);
                intake.Intakeon();;
                elevator.ElevatorSpeedfast();
                break;

            case EJECT:
                telemetry.addData("Collector State",ringCollectorState);
                intake.IntakeReverse();;
                elevator.Elevatorbackup();
                break;

        }


    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    //===================================================================
    // Helper Methods
    //==================================================================

    void debounce(long debounceTime){
        try {
            Thread.sleep(debounceTime);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


}
