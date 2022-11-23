package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@Autonomous(name = "Mod Autonomous", group = "Testing")
public class ModAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.resetEncoders();

        waitForStart();

        robot.move(690,0,0.7);

        robot.turnToAngle(50, 0.35);

        wait(500);

        robot.turnToAngle(-83, 0.35);

        //while (robot.colorSensor.blue() < 500) {
            //robot.setPower(0.4,0.4,0.4,0.4);
        //}
        robot.stopMotors();

        robot.move(210, -83, 0.7);

        wait(500);

        robot.reverse(300, -85,-0.5);

        robot.turnToAngle(-83, 0.35);

        wait(500);

        robot.turnToAngle(50, 0.35);

        robot.move(100, 50, 0.7);

        while (robot.getHeading() > -83) {
            robot.tankRight(0.35);
        }
        robot.stopMotors();

        stop();
    }

    public void wait(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}

