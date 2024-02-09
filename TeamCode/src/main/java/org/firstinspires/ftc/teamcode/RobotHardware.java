package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    private CRServo intakeServo;
    private Servo droneLaunch;

    private DistanceSensor rightDistanceSensor;
    private DistanceSensor leftDistanceSensor;
    private ColorSensor colorSensor;
    private AnalogInput armPotentiometer;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    static final double COUNTS_PER_MOTOR_REV = 560;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double DEFAULT_WHEEL_MOTOR_SPEED = .3;
    public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double MAX_POTENTIOMETER_ANGLE = 270;
    static final double ARM_MOTOR_COUNTS_PER_REV = 3500;
    public static final double ENCODER_COUNT_PER_DEGREE = ARM_MOTOR_COUNTS_PER_REV / 360;
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
        initServos();
        initSensors();
        initAnalogInput();
        initArmMotors();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
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

        // Set wheel motors to not resist turning when motor is stopped.
        leftFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRearWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Use STOP_AND_RESET_ENCODER to keep wheels from moving in unexpected ways
        // during subsequent runs of an OpMode.
        setRunModeForAllWheels(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Initialize all servos.
     */
    private void initServos() {
        // Define and initialize ALL installed servos.
        droneLaunch = myOpMode.hardwareMap.get(Servo.class, "DroneLaunch");
        intakeServo = myOpMode.hardwareMap.get(CRServo.class, "IntakeServo");
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
        armPotentiometer = myOpMode.hardwareMap.get(AnalogInput.class, "potentiometer");
    }

    private void initArmMotors() {
        rightArm = myOpMode.hardwareMap.get(DcMotor.class, "RArm");
        leftArm = myOpMode.hardwareMap.get(DcMotor.class, "LArm");
        rightArm.setDirection(DcMotor.Direction.FORWARD);
        leftArm.setDirection(DcMotor.Direction.FORWARD);
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
        setRunModeForAllWheels(DcMotor.RunMode.RUN_USING_ENCODER);
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
        leftArm.setPower(power);
        rightArm.setPower(power);
        //This telemetry reports when just manually moving arm...why?
        //myOpMode.telemetry.addData("Potentiometer Voltage", this.getPotentiometerVoltage());
        //myOpMode.telemetry.update();
    }

    public void setIntakeServoIntake() {
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeServo.setPower(1);
    }
    public void setIntakeServoOutake() {
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo.setPower(1);
    }
    public void setIntakeServoStop() {
        intakeServo.setPower(0);
    }


    /**
     * Move XYZ servo so that drone is released.
     */
    public void releaseDrone() {
        droneLaunch.setPosition(0.5);
    }
    public void resetDrone() {
        droneLaunch.setPosition(0.9);
    }

    public double getPotentiometerVoltage(){
        return armPotentiometer.getVoltage();
    }
    public double getMaxPotentiometerVoltage() {
        return armPotentiometer.getMaxVoltage();
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
        autoDriveRobot(-25,-25);
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
    /**
     * Move arm up or down until it gets to potentiometer's targetAngle.
     * @param targetAngle
     */
    public void setArmPositionUsingAngle(double targetAngle) {
        double currentAngle = getPotentiometerAngle();
        setRunModeForAllArms(DcMotor.RunMode.RUN_USING_ENCODER);

        //Ex. If currentAngle is 100 degrees, and targetAngle is 200 degrees,
        //    we want to move arm up.
        //    If currentAngle is 200 degrees, and targetAngle is 100 degrees,
        //    we want to move arm down.
        if (currentAngle < targetAngle) {
            myOpMode.telemetry.addData("Initial Angle", currentAngle);
            myOpMode.telemetry.addData("Initial Voltage", getPotentiometerVoltage());
            Telemetry.Item currentTelemetryItem = setupArmPositionTelemetry("Target Angle", targetAngle, "Current Angle", currentAngle);
            setArmPower(ARM_UP_POWER);

            while (myOpMode.opModeIsActive() && getPotentiometerAngle() < targetAngle) {
                currentTelemetryItem.setValue(getPotentiometerAngle());
                myOpMode.telemetry.update();
            }

            setArmPower(0); //Whoa
            myOpMode.telemetry.addData("Ending Angle", getPotentiometerAngle());
            myOpMode.telemetry.addData("Ending Voltage", getPotentiometerVoltage());
            myOpMode.telemetry.update();
        }
        else if (currentAngle > targetAngle) {
            myOpMode.telemetry.addData("Initial Angle", currentAngle);
            myOpMode.telemetry.addData("Initial Voltage", getPotentiometerVoltage());
            Telemetry.Item currentTelemetryItem = setupArmPositionTelemetry("Target Angle", targetAngle, "Current Angle", currentAngle);
            setArmPower(ARM_DOWN_POWER);

            while (myOpMode.opModeIsActive() && getPotentiometerAngle() > targetAngle) {
                currentTelemetryItem.setValue(getPotentiometerAngle());
                myOpMode.telemetry.update();
            }

            setArmPower(0); //Whoa
            myOpMode.telemetry.addData("Ending Angle", getPotentiometerAngle());
            myOpMode.telemetry.addData("Ending Voltage", getPotentiometerVoltage());
            myOpMode.telemetry.update();
        }
    }

    /**
     * Return calculated angle corresponding to potentiometer's current voltage.
     * Example: If the potentiometer's maximum voltage is 3.3,
     *          and it's maximum angle is 270 degrees,
     *          and the current voltage is 1.65
     *          then the current angle is 1.65 * 270 / 3.3 = 135 degrees.
     * @return angle
     */
    public double getPotentiometerAngle() {
        return armPotentiometer.getVoltage() * MAX_POTENTIOMETER_ANGLE / this.getMaxPotentiometerVoltage();
    }

    public void setRunModeForAllArms(DcMotor.RunMode runMode) {
        leftArm.setMode(runMode);
        rightArm.setMode(runMode);
    }

    /**
     * Set up telemetry to output this:
     *    <targetCaption> : <targetValue>
     *    <currentCaption> : <currentValue>
     * The Telemetry.Item for the currentValue is returned so the caller can keep updating it
     * using setValue().
     *
     * @param targetCaption
     * @param targetValue
     * @param currentCaption
     * @param currentValue
     * @return currentItem
     */
    private Telemetry.Item setupArmPositionTelemetry(String targetCaption, double targetValue, String currentCaption, double currentValue) {
        Telemetry telemetry = myOpMode.telemetry;
        telemetry.addData(targetCaption, targetValue);
        Telemetry.Item currentItem = telemetry.addData(currentCaption, currentValue);
        telemetry.update(); //Allow driver station to be cleared before display.
        telemetry.setAutoClear(false); //Henceforth updates should not clear display.

        return currentItem;
    }
    public void adjustArmAngleUsingEncoder(double angle) {
        int encoderCountAdjustment = (int) (ENCODER_COUNT_PER_DEGREE * angle);

        //The arms *should* be parallel, but calculate new targets for both arms anyway.
        int leftArmTargetEncoderCount = leftArm.getCurrentPosition() + encoderCountAdjustment;
        int rightArmTargetEncoderCount = rightArm.getCurrentPosition() + encoderCountAdjustment;

        leftArm.setTargetPosition(leftArmTargetEncoderCount);
        rightArm.setTargetPosition(rightArmTargetEncoderCount);
        setRunModeForAllArms(DcMotor.RunMode.RUN_TO_POSITION);

        //Set up telemetry
        myOpMode.telemetry.setAutoClear(false);
        Telemetry.Item leftArmItem = myOpMode.telemetry.addData("Left Arm", leftArm.getCurrentPosition());
        Telemetry.Item rightArmItem = myOpMode.telemetry.addData("Right Arm", rightArm.getCurrentPosition());
        myOpMode.telemetry.update();

        setArmPower(ARM_DOWN_POWER); //Using ARM_DOWN_POWER for now.

        // Update telemetry for as long as the wheel motors isBusy().
        while (leftArm.isBusy() && rightArm.isBusy()) {
            leftArmItem.setValue(leftArm.getCurrentPosition());
            rightArmItem.setValue(rightArm.getCurrentPosition());
            myOpMode.telemetry.update();
        }

        myOpMode.telemetry.setAutoClear(true);
    }

    public void manuallyMoveArm(double power) {
        setRunModeForAllArms(DcMotor.RunMode.RUN_USING_ENCODER);
        setArmPower(power);
    }
}