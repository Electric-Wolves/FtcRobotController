package org.firstinspires.ftc.teamcode.robot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp(name = "ImuStrafing", group = "Testing")
public class ImuStrafing extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap);

        while (opModeIsActive()) {
            robot.leftAutoStrafe(0, 0.4);

            wait(2500);

            robot.rightAutoStrafe(0, 0.4);
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
