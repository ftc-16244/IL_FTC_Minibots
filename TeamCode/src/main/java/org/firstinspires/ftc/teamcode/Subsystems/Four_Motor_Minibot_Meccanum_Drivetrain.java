package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class  Four_Motor_Minibot_Meccanum_Drivetrain {
        // Define hardware objects
        public DcMotor      leftFront   = null;
        public DcMotor      rightFront  = null;
        public DcMotor      leftRear    = null;
        public DcMotor      rightRear   = null;
        public BNO055IMU    imu         = null;

        // List constants
        public static final double COUNTS_PER_DRIVE_MOTOR_REV   = 1120;         // Nevrest 40's
        public static final double DRIVE_REDUCTION              = 16.0/24.0; // speed up - motor 24T wheel 16T
        public static final double WHEEL_DIAMETER_INCHES        = 60.0/25.4;      //Nexus 60mm wheels
        public static final double COUNTS_PER_INCH = (COUNTS_PER_DRIVE_MOTOR_REV *DRIVE_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * 3.1415);



        // Constructor
        public Four_Motor_Minibot_Meccanum_Drivetrain() {

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
            leftRear = hwMap.get(DcMotor.class, "Left_rear");
            rightRear = hwMap.get(DcMotor.class, "Right_rear");

            // For Mecanum , #5  and #6
            //leftFront.setDirection(DcMotor.Direction.REVERSE);
            //rightFront.setDirection(DcMotor.Direction.FORWARD);
            //rightRear.setDirection(DcMotor.Direction.FORWARD);
            //leftRear.setDirection(DcMotor.Direction.REVERSE);

            // For Mecanum #4
           // leftFront.setDirection(DcMotor.Direction.REVERSE);
            //rightFront.setDirection(DcMotor.Direction.FORWARD);
            //rightRear.setDirection(DcMotor.Direction.REVERSE);
            //leftRear.setDirection(DcMotor.Direction.FORWARD);

            // For Mecanum #3
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.REVERSE);
            leftRear.setDirection(DcMotor.Direction.FORWARD);


            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
                leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            }

        }



}


