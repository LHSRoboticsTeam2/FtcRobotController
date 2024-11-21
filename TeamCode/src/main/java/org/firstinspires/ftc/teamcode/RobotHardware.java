/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * Instead of each Op Mode class redefining the robot's hardware resources within its implementation,
 * This RobotHardware class has a given robot's component resources defined and set up all in one place.
 * It also has convenience methods like getSensorDistance(), setArmPower(), setHandPosition(), etc. that work for that robot.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 */
public class RobotHardware {

    /* Declare OpMode members. */
    private final LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.

    // Define all the HardwareDevices (Motors, Servos, etc.). Make them private so they can't be accessed externally.
    private DcMotor leftArm;
    private DcMotor rightArm;
    private WebcamName webCam;
    private Servo horizontalLift;
    private Servo grabberArmRight;
    private Servo grabberArmLeft;
    private Servo grabber;
    private DcMotor slideLeft;
    private DcMotor slideRight;
    private ColorSensor colorSensor;

    // Hardware device constants.  Make them public so they can be used by the calling OpMode, if needed.
    static final double COUNTS_PER_MOTOR_REV = 560;     // Assumes 20:1 gear reduction
    // See https://docs.revrobotics.com/duo-control/sensors/encoders/motor-based-encoders
    static final double ENCODER_COUNT_PER_DEGREE = COUNTS_PER_MOTOR_REV / 360;

    static final double DEFAULT_MOTOR_SPEED = .4;
    static final double  HEADING_THRESHOLD = 1.0 ; // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    static final double MID_SERVO       =  0.5 ;
    static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    static final double ARM_UP_POWER    =  0.45 ;
    static final double ARM_DOWN_POWER  = -0.45 ;
    static final double MAX_POTENTIOMETER_ANGLE = 270;

    static final double SENSOR_DISTANCE_OUT_OF_RANGE = 20;

    //Update these IMU parameters for your robot.
    static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
    /**
     * You can set the arm positions using angles and/or potentiometer voltage.
     * Tune these values for your robot's actual values.
     */
    static final double ARM_PARKED_ANGLE = 0;
    static final double ARM_PIXEL_PICKUP_ANGLE = 200;
    static final double ARM_BACKDROP_ANGLE = 100;
    static final double ARM_PARKED_VOLTAGE = 0;
    static final double ARM_PIXEL_PICKUP_VOLTAGE = 3;
    static final double ARM_BACKDROP_VOLTAGE = 1.4;

    /**
     * The one and only constructor requires a reference to a LinearOpMode.
     * @param opmode
     */
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Call init() to initialize all the robot's hardware.
     */
    public void init() {
        initServos();
        initSlideMotors();
//        initDistanceSensors();
//        initColorSensor();
//        initAnalogInputs();
//        initIMU();

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Call shutDown() to stop and close all the robot's hardware.
     */
    /**
     * Initialize all servos.
     */
    private void initServos() {
        // Define and initialize ALL installed servos.
        horizontalLift = myOpMode.hardwareMap.get(Servo.class, "horizontalLift");
        grabberArmRight = myOpMode.hardwareMap.get(Servo.class, "intakePlaneRight");
        grabberArmLeft = myOpMode.hardwareMap.get(Servo.class, "intakePlaneLeft");
        grabber = myOpMode.hardwareMap.get(Servo.class, "grabber");

        horizontalLift.setPosition(MID_SERVO);

        grabberArmLeft.setPosition(MID_SERVO);
        grabberArmRight.setPosition(MID_SERVO);
        grabberArmLeft.setDirection(Servo.Direction.FORWARD);
        grabberArmRight.setDirection(Servo.Direction.REVERSE);
        grabber.setPosition(MID_SERVO);
    }

    private void initSlideMotors() {
        slideRight = myOpMode.hardwareMap.get(DcMotor.class, "SlideRight");
        slideLeft = myOpMode.hardwareMap.get(DcMotor.class, "SlideLeft");

        slideRight.setDirection(DcMotorSimple.Direction.FORWARD);
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Initialize distance sensor(s).
     */
    private void initColorSensor() {
        colorSensor = myOpMode.hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    /**
     * Return robot's current heading (yaw) in degrees.
     * @return heading in degrees
     */

    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        horizontalLift.setPosition(MID_SERVO + offset);
        grabberArmLeft.setPosition(MID_SERVO - offset);
    }


    /**
     * Return all color values from Color Sensor.
     * @return RGBAcolors
     */
/*
    public RGBAColors getSensorColors() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        int alpha = colorSensor.alpha();

        return new RGBAColors(red, green, blue, alpha);
    }
           while (myOpMode.opModeIsActive() && getPotentiometerAngle() > targetAngle) {
                currentTelemetryItem.setValue(getPotentiometerAngle());
                myOpMode.telemetry.update();
            }

            setArmPower(0); //Whoa
        }
    }

*/
    public void extendHorizontalLift (float position){
        horizontalLift.setPosition(position);
    }
    public void openGrabber() {
        grabber.setPosition(1);
    }
    public void closeGrabber() {
        grabber.setPosition(0);
    }

    public void moveSlideUp() {
        slideRight.setPower(1);
        slideLeft.setPower(1);
    }


    public void moveSlideDown() {
        slideRight.setPower(-1);
        slideLeft.setPower(-1);
    }

    public void stopSlide() {
        slideRight.setPower(0);
        slideLeft.setPower(0);
    }

    public void grabberArmForward() {
        grabberArmRight.setPosition(1);
        grabberArmLeft.setPosition(1);
    }

    public void grabberArmBackward() {
        grabberArmRight.setPosition(0);
        grabberArmLeft.setPosition(0);
    }
}
