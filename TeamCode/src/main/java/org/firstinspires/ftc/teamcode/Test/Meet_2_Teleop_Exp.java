package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Enums.DriveSpeedState;
import org.firstinspires.ftc.teamcode.Enums.RingCollectionState;
import org.firstinspires.ftc.teamcode.Subsystems.Debouce;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain_v3;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Wobblegoal;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name="Meet 1A Teleop", group="Teleop")
//@Disabled
public class Meet_2_Teleop_Exp extends OpMode {


    /* Declare OpMode members. */

    private ElapsedTime runtime     = new ElapsedTime();
    public Drivetrain_v3        drivetrain  = new Drivetrain_v3(true);   // Use subsystem Drivetrain
    public Shooter              shooter     = new Shooter();
    public Intake               intake      = new Intake();
    public Wobblegoal           wobble  = new Wobblegoal();
    public Elevator elevator    = new Elevator();
    public ElapsedTime gripperCloseTimer = new ElapsedTime();
    //public ElapsedTime debounceTimer = new ElapsedTime();
    private Debouce mdebounce = new Debouce();

    private DriveSpeedState  currDriveState;
    private RingCollectionState ringCollectorState;
    private double gripperCloseTime = 1.0;

    // Add in Vuforia Items

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY =
            "AQXVmfz/////AAABmXaLleqhDEfavwYMzTtToIEdemv1X+0FZP6tlJRbxB40Cu6uDRNRyMR8yfBOmNoCPxVsl1mBgl7GKQppEQbdNI4tZLCARFsacECZkqph4VD5nho2qFN/DmvLA0e1xwz1oHBOYOyYzc14tKxatkLD0yFP7/3/s/XobsQ+3gknx1UIZO7YXHxGwSDgoU96VAhGGx+00A2wMn2UY6SGPl+oYgsE0avmlG4A4gOsc+lck55eAKZ2PwH7DyxYAtbRf5i4Hb12s7ypFoBxfyS400tDSNOUBg393Njakzcr4YqL6PYe760ZKmu78+8X4xTAYSrqFJQHaCiHt8HcTVLNl2fPQxh0wBmLvQJ/mvVfG495ER1A";
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
   public void init() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 2");

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsUltimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsUltimateGoal);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        blueAllianceTarget.setLocation(OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));
        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90)));

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , -90)));
        redTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = 4.0f * mmPerInch;   // eg: Camera is 4 Inches in front of robot-center
        final float CAMERA_VERTICAL_DISPLACEMENT = 14.0f * mmPerInch;   // eg: Camera is 8 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 3.0f;     // eg: Camera is ON the robot's center line

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }







        /* Initialize the hardware variables.
        * The init() method of the hardware class does all the work here
        */
        drivetrain.init(hardwareMap);
        intake.init(hardwareMap);
        wobble.init(hardwareMap);
        elevator.init(hardwareMap);
        shooter.init(hardwareMap);

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
        turn  =  gamepad1.right_stick_x;
        // Combine drive and turn for blended motion.
        left  = drive + turn;
        right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
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
            debounce(700);
            telemetry.addData("Flipper Fwd", "Complete ");
            shooter.flipperBackward();
            debounce(700);
        }
        if (gamepad1.right_trigger > 0.25) {
            //shooter.flipperBackward();
            //telemetry.addData("Flipper Back", "Complete ");
            shooter.shootonePowerShots();
            telemetry.addData("SHooter Low for Power Shots", "Complete ");
        }

        // Gamepad 1 Bumpers - for Speed Control
        // set-up drive speed states on bumpers
        if (gamepad1.left_stick_button)
        {
            currDriveState = DriveSpeedState.DRIVE_FAST;
        }
        if (gamepad1.right_stick_button)
        {
            currDriveState =  DriveSpeedState.DRIVE_SLOW;
        }


        // Wobble Controls

        if (gamepad1.dpad_left) {
            wobble.GripperOpen();
            wobble.ArmExtend();
            // wobble.resetWobble();

            telemetry.addData("Ready to rab Wobble", "Complete ");
        }

        if (gamepad1.dpad_up){
            gripperCloseTimer.reset();
            wobble.GripperClose();
            while (gripperCloseTimer.time() < gripperCloseTime){

                // stall program so gripper can close
                // not necessary in Linear Opmode just in iterative
                //more than a couple seconds and this will trow error
            }


            wobble.ArmCarryWobble();
            //wobble.readyToGrabGoal();
           telemetry.addData("Carrying Wobble", "Complete ");
        }
        if (gamepad1.dpad_right) {
            wobble.GripperOpen();
            wobble.ArmExtend();

            telemetry.addData("Dropping Wobble", "Complete ");
        }
        if (gamepad1.dpad_down) {
            wobble.ArmContract();
            wobble.GripperOpen();
            wobble.LiftLower();

            telemetry.addData("Reset Wobble", "Complete ");
        }
        if (gamepad1.back){
            wobble.LiftRise();
        }

        //========================================
        // GAME PAD 2 Mainly Wobble
        //========================================
        if (gamepad2.dpad_left) {
            wobble.GripperOpen();
            wobble.ArmExtend();
            // wobble.resetWobble();

            telemetry.addData("Ready to rab Wobble", "Complete ");
        }

        if (gamepad2.dpad_up){
            wobble.GripperClose();
            wobble.ArmCarryWobble();
            //wobble.readyToGrabGoal();
            telemetry.addData("Carrying Wobble", "Complete ");
        }
        if (gamepad2.dpad_right) {
            wobble.GripperOpen();
            wobble.ArmExtend();

            telemetry.addData("Dropping Wobble", "Complete ");
        }
        if (gamepad2.dpad_down) {
            wobble.ArmContract();
            wobble.GripperOpen();

            telemetry.addData("Reset Wobble", "Complete ");
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
