package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;


@Autonomous(name="AutoRedFar7615", group="chad")
public class AutoRedFar7615 extends LinearOpMode {

    //


    Hardware_7615 robot = new Hardware_7615();
    ElapsedTime runtime = new ElapsedTime();
    //28 * 20 / (2ppi * 4.125)
    Double width = 16.0; //inches
    Integer cpr = 538; //counts per rotation
    Integer gearratio = 20;
    Double diameter = 4.0;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder

    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP

    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference

    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /

            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED = 0.25;

    static final double TURN_SPEED = 0.125;

    //rpm code
    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 1;

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        return 32767 / ticksPerSecond;
    }

    public static final double TICKS_PER_REV = 1;
    public static final double MAX_RPM = 1;
    public static final boolean RUN_USING_ENCODER = true;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));
    public static double kV = 1 / rpmToVelocity(MAX_RPM);
    public static double kA = 0;
    public static double kStatic = 0;
    //
    Double conversion = cpi * bias;
    Boolean exit = false;
    //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    //
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };
    private static final String LABEL_BULB = "2 Bulb";
    private static final String LABEL_BOLT = "1 Bolt";
    private static final String LABEL_PANEL = "3 Panel";
    private static final String VUFORIA_KEY =
            "AZOwKVH/////AAABmRI93BwZSUFfnXivQIefPeBdhCO45QKgXMq0A4xZofqrAwdq93Tq7pU22J9ipBpMD/16YmRIo8t37wnyw3SKHWJ/bRvM739haBQcHCPPiKbMAOfPA9+Q+iP/g3WceBzi5OV3wLmi32dHtHuZwb9fLqoozDICKkxO5WOGV6StkxvfCMgHme3o6Enw68WBLHFoVG1syLto7whZMo7huSNDuBXsfo2GDyT3rYiu/hkEyNqpe0jF2E6S8EcjzAVsXViEJM/OWgujs9woYkVgjN8JSLjZE5eGfg8OWpSC1VlDpWKXuSpCZ1VNWbedNLfeEU7ugu13tZR0arFIUHF+JHJSjVAISrwxw6QDe8WkgMQR40Fy";


    WebcamName webcamName = null;
    //vuforia = ClassFactory.getInstance().createVuforia(parameters);

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;


    public void runOpMode() {
        initGyro();

        robot.init(hardwareMap);
        webcamName = hardwareMap.get(WebcamName.class, "7615_Cam");


        //  parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        //  parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        //  parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        // vuforia = ClassFactory.getInstance().createVuforia(parameters);


        //
        // vuforia = ClassFactory.getInstance().createVuforia(parameters);


        if (tfod != null) {
            tfod.activate();

        }
        //THIS IS WHAT THE ROBOT WILL DO
        waitForStartify();

        if (opModeIsActive()) {
            while (opModeIsActive()) {



                        encoderDrive(.5, -24,  -24, 5);
                        encoderDrive(.5, -23,  23, 5);
                        encoderDrive(.5, -85,  -85, 5);
                        sleep(300000);





            }


        }
        if (tfod != null) {
            tfod.shutdown();
        }
        //
        //moveToPosition(-55.8, 0.5);
        //strafeToPosition(38.0, 0.5);
        //strafeToPosition(15.0, 0.5);
        //strafeToPosition(16.2, 0.5);
        //moveToPosition(13, 0.5);
        //
    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        robot.LRear.setTargetPosition(robot.LRear.getCurrentPosition() + move);
        robot.LFront.setTargetPosition(robot.LFront.getCurrentPosition() + move);
        robot.RRear.setTargetPosition(robot.RRear.getCurrentPosition() + move);
        robot.RFront.setTargetPosition(robot.RFront.getCurrentPosition() + move);
        //
        robot.LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        robot.LFront.setPower(speed);
        robot.LRear.setPower(speed);
        robot.RFront.setPower(speed);
        robot.RRear.setPower(speed);
        //
        while (robot.LFront.isBusy() && robot.RFront.isBusy() && robot.LRear.isBusy() && robot.RRear.isBusy()){
            if (exit){
                robot.RFront.setPower(0);
                robot.LFront.setPower(0);
                robot.RRear.setPower(0);
                robot.LRear.setPower(0);
                return;
            }
        }
        robot.RFront.setPower(0);
        robot.LFront.setPower(0);
        robot.RRear.setPower(0);
        robot.LRear.setPower(0);
        return;
    }
    //
    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection){
        //<editor-fold desc="Initialize">
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //</editor-fold>
        //
        if (speedDirection > 0){//set target positions
            //<editor-fold desc="turn right">
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
            //</editor-fold>
        }else{
            //<editor-fold desc="turn left">
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
            //</editor-fold>
        }
        //
        //<editor-fold desc="Go to position">
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            //
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        //
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            robot.LFront.setPower(0);
            robot.RFront.setPower(0);
            robot.LRear.setPower(0);
            robot.RRear.setPower(0);
        }
        //</editor-fold>
        //
        robot.LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        robot.LRear.setTargetPosition(robot.LRear.getCurrentPosition() - move);
        robot.LFront.setTargetPosition(robot.LFront.getCurrentPosition() + move);
        robot.RRear.setTargetPosition(robot.RRear.getCurrentPosition() + move);
        robot.RFront.setTargetPosition(robot.RFront.getCurrentPosition() - move);
        //
        robot.LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.LRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.RRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        robot.LFront.setPower(speed);
        robot.LRear.setPower(speed);
        robot.RFront.setPower(speed);
        robot.RRear.setPower(speed);
        //
        while (robot.LFront.isBusy() && robot.RFront.isBusy() && robot.LRear.isBusy() && robot.RRear.isBusy()){}
        robot.RFront.setPower(0);
        robot.LFront.setPower(0);
        robot.RRear.setPower(0);
        robot.LRear.setPower(0);
        return;
    }
    //
    /*
    A tradition within the Thunder Pengwins code, we always start programs with waitForStartify,
    our way of adding personality to our programs.
     */
    public void waitForStartify(){
        waitForStart();
    }
    //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){
        robot.LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        robot.LFront.setPower(input);
        robot.LRear.setPower(input);
        robot.RFront.setPower(-input);
        robot.RRear.setPower(-input);
    }

    //







    public void encoderDrive(double speed,

                             double leftInches, double rightInches,

                             double timeoutS) {

        int newLeftFrontTarget;

        int newRightFrontTarget;

        int newLeftRearTarget;

        int newRightRearTarget;





        // Ensure that the opmode is still active

        if (opModeIsActive()) {



            // Determine new target position, and pass to motor controller

            newLeftFrontTarget = robot.LFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

            newLeftRearTarget = robot.LRear.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);

            newRightFrontTarget = robot.RFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            newRightRearTarget = robot.RRear.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.LFront.setTargetPosition(newLeftFrontTarget);

            robot.LRear.setTargetPosition(newLeftRearTarget);

            robot.RFront.setTargetPosition(newRightFrontTarget);

            robot.RRear.setTargetPosition(newRightRearTarget);



            // Turn On RUN_TO_POSITION

            robot.LFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.LRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.RFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.RRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            // reset the timeout time and start motion.

            runtime.reset();

            robot.LFront.setPower(Math.abs(speed));

            robot.LRear.setPower(Math.abs(speed));

            robot.RFront.setPower(Math.abs(speed));

            robot.RRear.setPower(Math.abs(speed));



            // keep looping while we are still active, and there is time left, and both motors are running.

            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits

            // its target position, the motion will stop.  This is "safer" in the event that the robot will

            // always end the motion as soon as possible.

            // However, if you require that BOTH motors have finished their moves before the robot continues

            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            while (opModeIsActive() &&

                    (runtime.seconds() < timeoutS) &&

                    (robot.LFront.isBusy() && robot.LRear.isBusy() && robot.RFront.isBusy() && robot.RRear.isBusy())) {



                // Display it for the driver.

                telemetry.addData("Path1", "Running to %7d :%7d", newLeftFrontTarget, newRightFrontTarget, newLeftRearTarget, newRightRearTarget);

                telemetry.addData("Path2", "Running at %7d :%7d",



                        robot.LFront.getCurrentPosition(),

                        robot.LRear.getCurrentPosition(),

                        robot.RFront.getCurrentPosition(),

                        robot.RRear.getCurrentPosition());


            }



            // Stop all motion;

            robot.LFront.setPower(0);

            robot.RFront.setPower(0);

            robot.LRear.setPower(0);

            robot.RRear.setPower(0);



            // Turn off RUN_TO_POSITION

            robot.LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            sleep(250);   // optional pause after each move

        }

    }

    public void drive() {
        robot.LFront.setPower(.1);
        robot.RFront.setPower(.1);
        robot.LRear.setPower(.1);
        robot.RRear.setPower(.1);
    }


    public void turn_right() {
        robot.LFront.setPower(.1);
        robot.RFront.setPower(-.1);
        robot.LRear.setPower(.1);
        robot.RRear.setPower(-.1);
    }


    public void turn_left() {
        robot.LFront.setPower(-.2);
        robot.RFront.setPower(.2);
        robot.LRear.setPower(-.2);
        robot.RRear.setPower(.2);
    }

    public void halt() {
        robot.LFront.setPower(0);
        robot.RFront.setPower(0);
        robot.LRear.setPower(0);
        robot.RRear.setPower(0);
    }

}