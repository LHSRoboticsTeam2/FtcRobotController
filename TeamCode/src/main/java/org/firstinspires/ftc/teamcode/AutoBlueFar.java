package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


/**
 * This 2023-2024 OpMode shows a way to drive using encoders to park from a starting position.
 */
@Autonomous(name = "AutoBlueFar", group = "")
public class AutoBlueFar extends LinearOpMode {
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

            robot.turnToSpike(propPositionNumber, SpikeColor.BLUE);
            driveToBackDrop(propPositionNumber);

            //break

        }
    }
    private void driveToBackDrop(int propPositionNumber) {
        int ninetyDegreeDistance = 15;
        double turnSpeed = .2;

        if (propPositionNumber == 2) {
            robot.autoDriveRobot(ninetyDegreeDistance, ninetyDegreeDistance * -1, turnSpeed); //Turn left 90 degrees
            robot.autoDriveRobot(-37, -37); // Run backwards to the backdrop
            sleep(100000);
        } else if (propPositionNumber == 1) {
            robot.autoDriveRobot(15, 15); // back away
            robot.autoDriveRobot(10,-10); //turn left
            robot.autoDriveRobot(-35, -35);//move forward and park
        } else {
            robot.autoDriveRobot(35, 35); // back toward close wall
        }
    }

}   // end class