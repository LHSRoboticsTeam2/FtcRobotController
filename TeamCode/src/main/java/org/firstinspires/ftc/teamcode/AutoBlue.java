package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous (name = "blue Auto")
public class AutoBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(this);
        RobotWheels robotWheels = new RobotWheels(this, robot);
        robot.init();
        robotWheels.init();

        waitForStart();
        // robot starts next to the wall, then moves forward, turns, then moves forward to the net zone

        if (opModeIsActive()) {
            robotWheels.autoDriveRobot(70, 70);
            //robot.moveSlideUp();
            //robot.moveSlideDown();

           // robot.grabberArmForward();
            //robot.openGrabber();
            //robot.moveSlideDown();
            //robot.grabberArmBackward();
        }
    sleep(7000);


    }
}

