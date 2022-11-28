package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@Autonomous(name = "Mod Autonomous", group = "Testing")
public class ModAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Thread slideLiftRaise, slideLiftLower;

        slideLiftRaise = new Thread() {
          @Override
          public void run() {
              robot.raiseLift(605, 0.85);

              robot.move(17, robot.getHeading(), 0.4);

              robot.lowerLift(500, -0.3);

              robot.openGripper();
          }
        };

        slideLiftLower = new Thread() {
          @Override
          public void run() {
              robot.lowerLift(100, -0.85);
          }
        };

        robot.closeGripper();

        waitForStart();

        slideLiftRaise.start();

        robot.move(615, robot.getHeading(), 0.8);

        while (robot.getHeading() < 49) {
            robot.tankLeft(0.4);
        }
        robot.stopMotors();

        robot.move(73, robot.getHeading(), 0.7);

        wait(4000);

        slideLiftLower.start();

        robot.reverse(160, robot.getHeading(), -0.5);

        robot.turnToAngle(-85, 0.4);

        robot.openGripper();

        // Repeat until target/goal is met
        while (robot.colorSensor.red() < 6000) {
            // If it is too far right, adjust
            if (robot.getHeading() < -85) {
                robot.setPower(0.5 - 0.05, 0.5 + 0.05, 0.5 - 0.05, 0.5 + 0.05);
            }
            // If it is too far left, adjust
            else if (robot.getHeading() > -85) {
                robot.setPower(0.5 + 0.05, 0.5 - 0.05, 0.5 + 0.05, 0.5 - 0.05);
            }
            // Maintain otherwise
            else {
                robot.setPower(0.5, 0.5, 0.5, 0.5);
            }
        }
        robot.stopMotors();

        wait(1000);

        while (robot.getCenterDist() > 75) {
            // If it is too far right, adjust
            if (robot.getHeading() < -85) {
                robot.setPower(0.5 - 0.05, 0.5 + 0.05, 0.5 - 0.05, 0.5 + 0.05);
            }
            // If it is too far left, adjust
            else if (robot.getHeading() > -85) {
                robot.setPower(0.5 + 0.05, 0.5 - 0.05, 0.5 + 0.05, 0.5 - 0.05);
            }
            // Maintain otherwise
            else {
                robot.setPower(0.5, 0.5, 0.5, 0.5);
            }
        }
        robot.stopMotors();

        robot.closeGripper();
    }

    public void wait(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}

