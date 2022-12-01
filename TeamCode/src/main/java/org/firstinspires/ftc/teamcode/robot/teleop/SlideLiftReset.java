package org.firstinspires.ftc.teamcode.robot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp (name = "SlideLiftReset", group = "util")
public class SlideLiftReset extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);

        waitForStart();

        telemetry.addLine("You are now in unlocked reset mode.");
        telemetry.update();

        while (opModeIsActive()) {
            while (gamepad2.left_stick_y != 0) {
                robot.slideLift.setPower(-gamepad2.left_stick_y);
            }

            robot.slideLift.setPower(0);
        }

        stop();
    }
}
