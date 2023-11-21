package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Teleop_7615", group="Iterative Opmode")
//@Disabled
public class Teleop_7615 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Hardware_7615 robot = new Hardware_7615();   // Use a 7615's hardware

    boolean isLeftBumper1Pressed = false;
    boolean isRightBumper1Pressed = false;
    boolean isIntakeOn = false;

    //Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
    }

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
    }

    //code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
    }

    Orientation angles;

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x * .5;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.RFront.setPower(v1);
        robot.LFront.setPower(v2);
        robot.RRear.setPower(v3);
        robot.LRear.setPower(v4);

        double elvatorUpSpeed = .5;
        double elvatorDownSpeed = .2;

        if (gamepad2.x) {
            robot.LArm.setPower(1);
            robot.RArm.setPower(1);
        }
        else if (gamepad2.b) {
            robot.LArm.setPower(-1);
            robot.RArm.setPower(-1);
        }

        else {
            robot.LArm.setPower(0);
            robot.RArm.setPower(0);
        }

        if (gamepad2.left_bumper) {
            robot.LeftGrab.setPosition(.1);

        }

        if (gamepad2.right_bumper) {
            robot.LeftGrab.setPosition(1);

        }

        if (gamepad2.y) {
            robot.RightPush.setPosition(.9);

        }

        if (gamepad2.a) {
            robot.RightPush.setPosition(.1);

        }



/*
        if (gamepad2.dpad_left) {
            //robot.Snoopy.setPosition(.7);

        }

        if (gamepad2.dpad_right){
            //robot.Snoopy.setPosition(.3);

        }

        if (gamepad1.dpad_up){
            //robot.Lulu.setPosition(.7);
            //robot.Fluffy.setPosition(0.2);
        }

        if(gamepad1.dpad_down){
            //robot.Lulu.setPosition(.2);
            //robot.Fluffy.setPosition(.7);

        }

        //if(gamepad1.dpad_left) {
            //robot.Kenny.setPosition(.6);

        //}
        //if(gamepad1.dpad_right){
            //robot.Kenny.setPosition(.1);
        //}



        while (gamepad1.y) {
            robot.Elevator1.setPower(elvatorUpSpeed);
            robot.Elevator2.setPower(elvatorUpSpeed);
            robot.Elevator3.setPower(elvatorUpSpeed);
        }

        while (gamepad1.x) {
            robot.Elevator1.setPower(-1);
            robot.Elevator2.setPower(-1);
            robot.Elevator3.setPower(-1);
        }

        while (gamepad1.a) {
            robot.Elevator1.setPower(-elvatorDownSpeed);
            robot.Elevator2.setPower(-elvatorDownSpeed);
            robot.Elevator3.setPower(-elvatorDownSpeed);
        }


        if ( !gamepad1.y && !gamepad1.a) {
            robot.Elevator1.setPower(0);
            robot.Elevator2.setPower(0);
            robot.Elevator3.setPower(0);
        }

        double elevatorUp       = .25;
        double elevatorDown     = .8;

        if (gamepad1.dpad_up) {
            robot.ElevatorLeft.setPosition(elevatorUp);
            robot.ElevatorRight.setPosition(elevatorUp);
        }

        if (gamepad1.dpad_down) {
            robot.ElevatorLeft.setPosition(elevatorDown);
            robot.ElevatorRight.setPosition(elevatorDown);
        }

        if (gamepad1.dpad_left){
            robot.IntakeLeft.setPosition(.5);
            robot.IntakeRight.setPosition(.5);
        }

        if (gamepad1.dpad_right) {
            robot.IntakeLeft.setPosition(.9);
            robot.IntakeRight.setPosition(.9);
        }

        if (!isRightBumper1Pressed && gamepad1.right_bumper)  {
            if (isIntakeOn) {
                robot.Intake.setPower(1);
                isIntakeOn = false;
            } else {
                robot.Intake.setPower(0);
                isIntakeOn = true;
            }
            isRightBumper1Pressed = true;
        } else if (!gamepad1.right_bumper){
            isRightBumper1Pressed = false;
        }

        if (!isLeftBumper1Pressed && gamepad1.left_bumper)  {
            if (isIntakeOn) {
                robot.Intake.setPower(-.25);
                isIntakeOn = false;
            } else {
                robot.Intake.setPower(0);
                isIntakeOn = true;
            }
            isLeftBumper1Pressed = true;
        } else if (!gamepad1.left_bumper){
            isLeftBumper1Pressed = false;
        }


*/
    }






    String formatAngle (AngleUnit angleUnit,double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees ( double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    //Code to run ONCE after the driver hits STOP
    @Override public void stop () {
    }


}