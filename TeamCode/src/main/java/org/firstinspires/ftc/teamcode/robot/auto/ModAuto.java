package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@Autonomous(name = "Mod Autonomous", group = "Testing")
public class ModAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Thread slideLiftRaise;

        slideLiftRaise = new Thread() {
          @Override
          public void run() {
              robot.raiseLift(605, -0.85);

              robot.move(20, robot.getHeading(), 0.4);

              robot.lowerLift(500, 0.3);

              robot.openGripper();
          }
        };

        robot.closeGripper();

        waitForStart();

        robot.closeGripper();

        slideLiftRaise.start();

        robot.move(615, robot.getHeading(), 0.8);

        while (robot.getHeading() < 45) {
            robot.tankLeft(0.4);
        }
        robot.stopMotors();

        robot.move(75, robot.getHeading(), 0.7);

        wait(500);

        /*while (robot.getCenterDist() > 200) {
            robot.tankLeft(0.3);
        }*/
        robot.stopMotors();


        /*

        while (robot.getFirstDist() >= 150) {
            robot.tankLeft(0.4);
        }
        robot.stopMotors();

        while (robot.getCenterDist() >= 150) {
            robot.tankRight(0.4);
        }
        robot.stopMotors();
        ?
         */

        /*

        slideLiftRaise.start();

        robot.move(615, 0, 0.8);

        robot.closeGripper();

        while (robot.getHeading() < 51) {
            robot.tankLeft(0.4);
        }
        robot.stopMotors();
        ?
         */

    }

    public void wait(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}

