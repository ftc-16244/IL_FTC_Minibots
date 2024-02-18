package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Two_Motor_REV_Core_HEX_Drivetrain {
        // Define hardware objects
        public DcMotor leftFront = null;
        public DcMotor rightFront = null;
        public BNO055IMU imu = null;

        // List constants
        public static final double COUNTS_PER_DRIVE_MOTOR_REV = 288;     // Core Hex motors
        public static final double DRIVE_REDUCTION = 1.0; //Stright off the motor
        public static final double WHEEL_DIAMETER_INCHES = 90.0/25.4;  //90mm converted to inches
        public static final double COUNTS_PER_INCH = (COUNTS_PER_DRIVE_MOTOR_REV *DRIVE_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);
        public static final double DRIVE_SPEED = 0.9;
        private static final double TURN_SPEED = 0.5;
        private boolean inTeleOp;
        private ElapsedTime runtime = new ElapsedTime();


        // Contructor for Drivetrain

        public Two_Motor_REV_Core_HEX_Drivetrain(){
        }

        // initialize drivetrain components, assign names for driver station config, set directions
        // and encoders if needed.
        public void init(HardwareMap hwMap, boolean inTeleOp) {

            // initialize the imu first.
            // Note this in NOT IMU calibration.
            imu = hwMap.get(BNO055IMU.class, "imu");



            // initialize al the drive motors
            leftFront = hwMap.get(DcMotor.class, "Left_front");
            rightFront = hwMap.get(DcMotor.class, "Right_front");

            // The Core HEX motors sometimes get their direction backwards
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            rightFront.setDirection(DcMotor.Direction.REVERSE);

            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // not in teleop means autonomous so encoders are needed
            // IMU us also needed
            if (!inTeleOp) {

                leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            } else {
                // for InTeleop we don't need encoders because driver controls
                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                //rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            }

        }



}


