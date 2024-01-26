package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This 2023-2024 OpMode shows a way to drive using encoders to park from a starting position.
 */
@Autonomous(name = "AutoRedFar", group = "")
public class AutoRedFar extends LinearOpMode {
    RobotHardware robot;
    @Override
    public void runOpMode() {

        robot = new RobotHardware(this);
        robot.init();

        int propPositionNumber;

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {

            robot.driveToSpike(SpikeColor.RED, 550);
            telemetry.setAutoClear(false);
            propPositionNumber = robot.getSpikeObjectPosition();

            robot.turnToSpike(propPositionNumber, SpikeColor.RED);
            driveToBackDrop(propPositionNumber);
        }
    }
    private void driveToBackDrop(int propPositionNumber) {
        int ninetyDegreeDistance = 15;
        double turnSpeed = .2;

        if (propPositionNumber == 2) {
            robot.autoDriveRobot(ninetyDegreeDistance * -1, ninetyDegreeDistance, turnSpeed); //Turn right 90 degrees
            robot.autoDriveRobot(-37, -37); // Run backwards to the backdrop
        } else if (propPositionNumber == 1) {
            //robot.autoDriveRobot(-15,15); //turn right
            robot.autoDriveRobot(40, 40); // move backward to wall
        } else {
            robot.autoDriveRobot(15, 15); // back away
            robot.autoDriveRobot(-15,15); //turn right
            robot.autoDriveRobot(-35, -35); // move forward and park
        }
    }

}   // end class