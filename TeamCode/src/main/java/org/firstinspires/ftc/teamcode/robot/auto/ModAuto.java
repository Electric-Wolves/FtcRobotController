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
              robot.closeGripper();

              while (robot.ticksToMM(robot.slideLiftEncoder.getCurrentPosition()) < 605) {
                  robot.slideLift.setPower(-0.8);
                  robot.closeGripper();
              }
              robot.slideLift.setPower(0);

              robot.move(80, robot.getHeading(), 0.5);
          }
        };

        robot.closeGripper();

        waitForStart();

        slideLiftRaise.start();

        robot.move(615, 0, 0.8);

        robot.closeGripper();

        while (robot.getHeading() < 51) {
            robot.tankLeft(0.4);
        }
        robot.stopMotors();
    }

    public void wait(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}

