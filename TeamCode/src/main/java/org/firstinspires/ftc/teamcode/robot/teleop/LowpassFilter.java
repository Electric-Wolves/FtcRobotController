package org.firstinspires.ftc.teamcode.robot.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;

public class LowpassFilter extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine(String.format("Left Distance: %.2f mm", lpFilter(robot.getLeftMM())));
            telemetry.addLine(String.format("Center Distance: %.2f mm", lpFilter(robot.getCenterDist())));
            telemetry.update();
        }
    }

    public double lpFilter(double rawDist) {
        return ((rawDist * 2) - 1);
    }
}
