package org.firstinspires.ftc.teamcode.robot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp(name = "ServoTesting", group = "Testing")
public class ServoTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            robot.leftGrip.setPosition(1); // Opened
            robot.rightGrip.setPosition(0); // Opened

            wait(5000);

            robot.leftGrip.setPosition(0); // Closed
            robot.rightGrip.setPosition(0.95); // Closed

            wait(5000);
        }
    }

    public void wait(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}
