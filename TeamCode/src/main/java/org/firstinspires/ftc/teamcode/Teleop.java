package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Manually control robot", group = "")
public class Teleop  extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(this);
        RobotWheels wheels = new RobotWheels(this,robot);
        robot.init();
        wheels.init();

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            wheels.manuallyDriveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

            if (gamepad1.left_bumper) {
                robot.openGrabber();
            }
            else if (gamepad1.right_bumper) {
                robot.closeGrabber();
            }

            while (gamepad1.x) {
               robot.moveSlideUp();
            }
            while (gamepad1.y) {
                robot.moveSlideDown();
            }
            if (!gamepad1.x && !gamepad1.y) {
                robot.stopSlide();
            }

            robot.extendHorizontalLift(gamepad1.right_trigger);

            if (gamepad1.dpad_down)
            {
                robot.grabberArmForward();
            }
            else if (gamepad1.dpad_up)
            {
                robot.grabberArmBackward();
            }
            else {
                robot.grabberArmStop();
            }
        }
    }
}