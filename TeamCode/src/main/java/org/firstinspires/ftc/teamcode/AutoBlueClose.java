package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This 2023-2024 OpMode shows a way to drive using encoders to park from a starting position.
 */
@Autonomous(name = "AutoBlueClose", group = "")
public class AutoBlueClose extends LinearOpMode {
    RobotHardware robot;
    @Override
    public void runOpMode() {

        robot = new RobotHardware(this);
        robot.init();

        int propPositionNumber;

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {

            robot.driveToSpike(SpikeColor.BLUE, 850);
            telemetry.setAutoClear(false);
            propPositionNumber = robot.getSpikeObjectPosition();

            this.turnToSpike(propPositionNumber);
            //driveToBackDrop(propPositionNumber);
        }
    }

    private void turnToSpike(int propPositionNumber) {
        telemetry.addData("Op Mode",propPositionNumber);
        telemetry.update();

        double turnSpeed = .15;

        if (propPositionNumber == 2) {
            robot.autoDriveRobot(20,20); //Back away from spike
        } else if (propPositionNumber == 1) {
            robot.autoDriveRobot(16,-16, turnSpeed); //place pixel on spike 1
            robot.autoDriveRobot(3, 3); //back away from spike
        }
        //3
        else {
            robot.autoDriveRobot(-14, 14, turnSpeed); //place pixel on spike 3
            robot.autoDriveRobot(5,5); // back away from spike
        }
    }
    private void driveToBackDrop(int propPositionNumber) {
        int ninetyDegreeDistance = 15;
        double turnSpeed = .2;

        if (propPositionNumber == 2) {
            robot.autoDriveRobot(ninetyDegreeDistance, ninetyDegreeDistance * -1, turnSpeed); //Turn left 90 degrees
            robot.autoDriveRobot(100, 100); // Run backwards to the backdrop
        } else if (propPositionNumber == 1) {
            robot.autoDriveRobot(-12, 12); // turn right
            robot.autoDriveRobot(18, 18); // back away
            robot.autoDriveRobot(16, -16); //turn left
            robot.autoDriveRobot(100, 100); // drive to backdrop
        } else {
            robot.autoDriveRobot(1, -1); //turn left
            robot.autoDriveRobot(40, 40); // back toward close wall
            robot.autoDriveRobot(-100, -100); // drive to backdrop
        }
    }

}   // end class