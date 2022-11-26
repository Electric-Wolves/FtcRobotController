package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;

public class RedRightAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);

        telemetry.addLine("Robot Initialized. Press > to Run.");
        telemetry.update();

        waitForStart();
    }
}
