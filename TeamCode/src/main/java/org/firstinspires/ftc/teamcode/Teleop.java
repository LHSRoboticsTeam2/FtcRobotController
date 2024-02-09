package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Manually control robot", group = "")
public class Teleop  extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        robot.init();
        robot.resetDrone();

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            robot.manuallyDriveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if (gamepad1.a) {
                robot.releaseDrone();
            }
            else if (gamepad1.b) {
                robot.resetDrone();
            }

            if (gamepad1.x) {
                telemetry.addData("X Pressed", "");
                robot.manuallyMoveArm(0.3);
                //robot.adjustArmAngleUsingEncoder(-175);
            }
            else if (gamepad1.y) {
                telemetry.addData("Y Pressed", "");
                robot.manuallyMoveArm(-0.3);
            }
            else {
                //telemetry.addData("Neither X or Y Pressed", "");
                robot.manuallyMoveArm(0);
            }

            if (gamepad1.right_bumper) {
                robot.setIntakeServoIntake();
            }
            else if (gamepad1.left_bumper) {
                robot.setIntakeServoOutake();
            }
            else {
                robot.setIntakeServoStop();
            }

        }
    }
}