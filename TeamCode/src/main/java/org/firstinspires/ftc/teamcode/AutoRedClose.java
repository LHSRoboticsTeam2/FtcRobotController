package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;


/**
 * This 2023-2024 OpMode shows a way to drive using encoders to park from a starting position.
 */
@Autonomous(name = "AutoRedClose")
public class AutoRedClose extends LinearOpMode {

    @Override
    public void runOpMode() {

        RobotHardware robot = new RobotHardware(this);
        robot.init();

        int propPositionNumber;

        waitForStart();

        // Strictly-speaking, checking opModeIsActive() is not really needed here.
        if (opModeIsActive()) {
            robot.driveToSpike(SpikeColor.RED, 2000);
            telemetry.setAutoClear(false);
            propPositionNumber = robot.getSpikeObjectPosition();
            int pixelTurnDistance = 12;
            int ninetyDegreeDistance = 17;
            double turnSpeed = .2;
            if (propPositionNumber == 2) {
                robot.autoDriveRobot(5,5);
                robot.autoDriveRobot(ninetyDegreeDistance, ninetyDegreeDistance * -1, turnSpeed);
            } else if (propPositionNumber == 1) {
                robot.autoDriveRobot(pixelTurnDistance,pixelTurnDistance * -1, turnSpeed);
            }
            else {
                robot.autoDriveRobot(pixelTurnDistance * -1, pixelTurnDistance, turnSpeed);
            }
            telemetry.addData("Op Mode",propPositionNumber);
            telemetry.update();
        }

        robot.shutDown();
    }


}   // end class

