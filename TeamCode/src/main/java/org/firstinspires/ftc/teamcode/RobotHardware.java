package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Instead of each Op Mode class redefining the robot's hardware resources within its implementation,
 * This RobotHardware class has a given robot's component resources defined and set up all in one place.
 * It also has convenience methods like driveRobot(), setArmPower(), setHandPosition(), etc. that work for that robot.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 */
public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define all the HardwareDevices (Motors, Servos, etc.). Make them private so they can't be accessed externally.
    private DcMotor leftFrontWheel;
    private DcMotor rightFrontWheel;
    private DcMotor leftRearWheel;
    private DcMotor rightRearWheel;
    private DcMotor rightArm;
    private DcMotor leftArm;

    // Define other HardwareDevices as needed.
    private DcMotor armMotor;
    private Servo   leftHand;
    private Servo   rightHand;

    private DistanceSensor rightDistanceSensor;
    private DistanceSensor leftDistanceSensor;
    private ColorSensor colorSensor;
    private AnalogInput potentiometer;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DEFAULT_WHEEL_MOTOR_SPEED = .4;
    public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /**
     * The one and only constructor requires a reference to an OpMode.
     * @param opmode
     */
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Call init() to initialize all the robot's hardware.
     */
    public void init() {
        initWheelMotors();
        //initServos();
        initSensors();
        initAnalogInput();
        initArmMotors();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Call shutDown() to stop and close all the robot's hardware.
     */
    public void shutDown() {

    }
    /**
     * Initialize all the wheel motors.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    private void initWheelMotors()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontWheel  = myOpMode.hardwareMap.get(DcMotor.class, "LFront");
        rightFrontWheel = myOpMode.hardwareMap.get(DcMotor.class, "RFront");
        leftRearWheel = myOpMode.hardwareMap.get(DcMotor.class, "LRear");
        rightRearWheel = myOpMode.hardwareMap.get(DcMotor.class, "RRear");

        // To drive forward, most robots need the motors on one side to be reversed, because the axles point in opposite directions.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        leftRearWheel.setDirection(DcMotor.Direction.FORWARD);
        rightFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        rightRearWheel.setDirection(DcMotor.Direction.REVERSE);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set wheel motors to not resist turning when motor is stopped.
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Initialize all servos.
     */
    private void initServos() {
        // Define and initialize ALL installed servos.
        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");

        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */

    private void initSensors() {
        rightDistanceSensor = myOpMode.hardwareMap.get(DistanceSensor.class, "rightSensor");
        leftDistanceSensor = myOpMode.hardwareMap.get(DistanceSensor.class, "leftSensor");
        colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "colorSensor");
    }
    private void initAnalogInput() {
        potentiometer = myOpMode.hardwareMap.get(AnalogInput.class, "potentiometer");
    }

    private void initArmMotors() {
        rightArm = myOpMode.hardwareMap.get(DcMotor.class, "RArm");
        leftArm = myOpMode.hardwareMap.get(DcMotor.class, "LArm");
        rightArm.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setDirection(DcMotor.Direction.REVERSE);
    }


    /**
     * Drive robot to the targeted position designated by the passed leftInches and
     * rightInches, at the power specified by speed.
     * @param leftInches
     * @param rightInches
     * @param speed
     */
    public void autoDriveRobot(int leftInches, int rightInches, double speed) {
        int leftInchesToCPI = (int) (leftInches * COUNTS_PER_INCH);
        int rightInchesToCPI = (int) (rightInches * COUNTS_PER_INCH);

        int leftFrontTarget = leftFrontWheel.getCurrentPosition() + leftInchesToCPI;
        int leftRearTarget = leftRearWheel.getCurrentPosition() + leftInchesToCPI;
        int rightFrontTarget = rightFrontWheel.getCurrentPosition() + rightInchesToCPI;
        int rightRearTarget = rightRearWheel.getCurrentPosition() + rightInchesToCPI;

        leftFrontWheel.setTargetPosition(leftFrontTarget);
        leftRearWheel.setTargetPosition(leftRearTarget);
        rightFrontWheel.setTargetPosition(rightFrontTarget);
        rightRearWheel.setTargetPosition(rightRearTarget);

        setRunModeForAllWheels(DcMotor.RunMode.RUN_TO_POSITION);

        //Set up telemetry
        myOpMode.telemetry.setAutoClear(false);
        myOpMode.telemetry.addData("Heading", "Current Wheel Positions");
        Telemetry.Item leftFrontWheelItem = myOpMode.telemetry.addData("LF Wheel", leftFrontWheel.getCurrentPosition());
        Telemetry.Item leftRearWheelItem = myOpMode.telemetry.addData("LR Wheel", leftRearWheel.getCurrentPosition());
        Telemetry.Item rightFrontWheelItem = myOpMode.telemetry.addData("RF Wheel", rightFrontWheel.getCurrentPosition());
        Telemetry.Item rightRearWheelItem = myOpMode.telemetry.addData("RR Wheel", rightRearWheel.getCurrentPosition());
        myOpMode.telemetry.update();

        double absoluteSpeed = Math.abs(speed);
        setPowerAllWheels(absoluteSpeed);

        // Update telemetry for as long as the wheel motors isBusy().
        while (leftFrontWheel.isBusy() && leftRearWheel.isBusy() && rightFrontWheel.isBusy() && rightRearWheel.isBusy()) {
            leftFrontWheelItem.setValue(leftFrontWheel.getCurrentPosition());
            leftRearWheelItem.setValue(leftRearWheel.getCurrentPosition());
            rightFrontWheelItem.setValue(rightFrontWheel.getCurrentPosition());
            rightRearWheelItem.setValue(rightRearWheel.getCurrentPosition());
            myOpMode.telemetry.update();
        }

        //Robot has RUN_TO_POSITION.
        setPowerAllWheels(0); //Whoa.
        myOpMode.telemetry.setAutoClear(true);
    }

    /**
     * autoDriveRobot using DEFAULT_WHEEL_MOTOR_SPEED.
     * @param leftInches
     * @param rightInches
     */
    public void autoDriveRobot(int leftInches, int rightInches) {
        autoDriveRobot(leftInches, rightInches, DEFAULT_WHEEL_MOTOR_SPEED);
    }

    public double getRightPropDistanceInCM() {
        double distance = rightDistanceSensor.getDistance(DistanceUnit.CM);
        return distance;
    }

    public double getLeftPropDistanceInCM() {
        double distance = leftDistanceSensor.getDistance(DistanceUnit.CM);
        return distance;
    }
    /**
     * Set the RunMode for all wheel motors to the passed runMode.
     * @param runMode
     */
    public void setRunModeForAllWheels(DcMotor.RunMode runMode) {
        leftFrontWheel.setMode(runMode);
        leftRearWheel.setMode(runMode);
        rightFrontWheel.setMode(runMode);
        rightRearWheel.setMode(runMode);
    }

    /**
     * Set the Power for all wheels to the passed speed.
     * @param speed
     */
    public void setPowerAllWheels(double speed) {
        leftFrontWheel.setPower(speed);
        leftRearWheel.setPower(speed);
        rightFrontWheel.setPower(speed);
        rightRearWheel.setPower(speed);
    }

    /**
     * Drive robot according to passed stick inputs.
     * @param stick1X Value from stick 1's X axis
     * @param stick1Y Value from stick 1's Y axis
     * @param stick2X Value from stick 2's X axis
     */
    public void manuallyDriveRobot(double stick1X, double stick1Y, double stick2X) {
        double vectorLength = Math.hypot(stick1X, stick1Y);
        double robotAngle = Math.atan2(stick1Y, -stick1X) - Math.PI / 4;
        double rightXscale = stick2X * .5;
        final double rightFrontVelocity = vectorLength * Math.cos(robotAngle) + rightXscale;
        final double leftFrontVelocity = vectorLength * Math.sin(robotAngle) - rightXscale;
        final double rightRearVelocity = vectorLength * Math.sin(robotAngle) + rightXscale;
        final double leftRearVelocity = vectorLength * Math.cos(robotAngle) - rightXscale;
        // Use existing method to drive both wheels.
        setDrivePower(leftFrontVelocity, rightFrontVelocity, leftRearVelocity, rightRearVelocity);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     */
    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower) {
        // Output the values to the motor drives.
        leftFrontWheel.setPower(leftFrontPower);
        rightFrontWheel.setPower(rightFrontPower);
        leftRearWheel.setPower(leftRearPower);
        rightRearWheel.setPower(rightRearPower);
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    public void setArmPower(double power) {
        armMotor.setPower(power);
    }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);
    }

    /**
     * Move XYZ servo so that drone is released.
     */
    public void releaseDrone() {
        //Move whichever servo(?).
    }

    public double getPotentiometerVoltage(){
        return potentiometer.getVoltage();
    }
    public double getMaxPotentiometerVoltage(){
        return potentiometer.getMaxVoltage();
    }
    public RGBAColors getSensorColors() {
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();
        int alpha = colorSensor.alpha();
        myOpMode.telemetry.addData("RGB", red + ", " + green + ", " + blue);
        myOpMode.telemetry.update();
        return new RGBAColors(red, blue,green, alpha);
    }

    public int getSpikeObjectPosition() {
        double leftSensorDistance = getLeftPropDistanceInCM();
        double rightSensorDistance = getRightPropDistanceInCM();
        int positionNumber;
        double maximumSensorDistance = 20;
        if (leftSensorDistance > maximumSensorDistance && rightSensorDistance > maximumSensorDistance){
            positionNumber = 2;
        }
        else if (leftSensorDistance <= maximumSensorDistance) {
            positionNumber = 1;
        }
        else {
            positionNumber = 3;
        }
        myOpMode.telemetry.addData("getSpikeObjectPosition",positionNumber);
        myOpMode.telemetry.update();
        return positionNumber;
    }

    public void driveToSpike(SpikeColor color, int colorThreshold) {
        autoDriveRobot(-20,-20);
        setRunModeForAllWheels(DcMotor.RunMode.RUN_USING_ENCODER);
        setPowerAllWheels(-.05);
        RGBAColors colors;
        while (myOpMode.opModeIsActive()) {
            colors = getSensorColors();
            if (color == SpikeColor.BLUE && colors.getBlue() > colorThreshold){
                break;
            }
            else if(colors.getRed() > colorThreshold) {
                break;
            }
        }
        setPowerAllWheels(0);
    }

    public void turnToSpike(int propPositionNumber) {
        int pixelTurnDistance = 12;
        int ninetyDegreeDistance = 15;
        double turnSpeed = .2;
        if (propPositionNumber == 2) {
            autoDriveRobot(20,20);
            autoDriveRobot(ninetyDegreeDistance, ninetyDegreeDistance * -1, turnSpeed);
            autoDriveRobot(100, 100);
        } else if (propPositionNumber == 1) {
            autoDriveRobot(pixelTurnDistance,pixelTurnDistance * -1, turnSpeed);
            autoDriveRobot(3, 3);
            autoDriveRobot(-12, 12);
            autoDriveRobot(18,18);
            autoDriveRobot(16
                    , -16);
            autoDriveRobot(100,100);
        }
        //3

        else {
            autoDriveRobot(pixelTurnDistance * -1, pixelTurnDistance, turnSpeed);
            autoDriveRobot(5,5);
            autoDriveRobot(1,-1);
            autoDriveRobot(40,40);
            autoDriveRobot(-100, -100);
        }
        myOpMode.telemetry.addData("Op Mode",propPositionNumber);
        myOpMode.telemetry.update();
    }
}


