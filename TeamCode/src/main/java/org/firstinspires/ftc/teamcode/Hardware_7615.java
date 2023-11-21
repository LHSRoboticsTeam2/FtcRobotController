package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class
Hardware_7615 {
    public DcMotor LFront       = null;
    public DcMotor RRear        = null;
    public DcMotor RFront       = null;
    public DcMotor LRear        = null;
    //public DcMotor Ellie        = null;
    //public DcMotor Hazel       = null;
    //public DcMotor Molly       = null;
    //public DcMotor Finn       = null;
    public BNO055IMU imu        = null;
    public Servo RightGrab    = null;
    public Servo LeftGrab    = null;
    public Servo RightPush    = null;
    public Servo LeftPush    = null;



    //  public CRServo Lift    = null;


    public DcMotor RArm  = null;
    public DcMotor LArm   = null;


    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware_7615(){
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        LFront      = hwMap.get(DcMotor.class, "LFront");
        RFront      = hwMap.get(DcMotor.class, "RFront");
        LRear       = hwMap.get(DcMotor.class, "LRear");
        RRear       = hwMap.get(DcMotor.class, "RRear");
        //Ellie       = hwMap.get(DcMotor.class, "Ellie");
        //Hazel       = hwMap.get(DcMotor.class, "Hazel");
        //Molly       = hwMap.get(DcMotor.class, "Molly");
        //Finn       = hwMap.get(DcMotor.class, "Finn");
        LArm   =hwMap.get(DcMotor.class, "LArm");
        RArm    =hwMap.get(DcMotor.class, "RArm");
        // Define and initialize ALL installed servos.

        RightGrab      =hwMap.get(Servo.class, "RightGrab");
        LeftGrab      =hwMap.get(Servo.class, "LeftGrab");
        RightPush      =hwMap.get(Servo.class, "RightPush");
        LeftPush      =hwMap.get(Servo.class, "LeftPush");



        //Ruby      =hwMap.get(Servo.class, "Ruby");
        //Fluffy      =hwMap.get(Servo.class, "Fluffy");
        //Lulu      =hwMap.get(Servo.class, "Lulu");
        //Kenny      =hwMap.get(Servo.class, "Kenny");
        //Define and initialize ALL installed sensors
        BNO055IMU.Parameters parameters             = new BNO055IMU.Parameters();
        parameters.angleUnit                        = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit                        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile              = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled                   = true;
        parameters.loggingTag                       = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu                                         =hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        LFront.setDirection(DcMotor.Direction.REVERSE);
        LRear.setDirection(DcMotor.Direction.FORWARD);
        RFront.setDirection(DcMotor.Direction.REVERSE);
        RRear.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        LFront.setPower(0);
        RFront.setPower(0);
        RRear.setPower(0);
        LRear.setPower(0);

        // May want to use RUN_USING_ENCODERS if encoders are installed.
        LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Hazel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Molly.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        // Set motor ZeroPower Behavior
        LFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        LRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}