package org.firstinspires.ftc.teamcode.robot.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp(name = "L1TeleOp", group = "L1")
public class DriverTeleOp extends LinearOpMode {
    final static double L = 170; // Distance between Encoder 0 and 1
    final static double B = 150; // Distance from midpoint of L to Encoder 2
    final static double R = 37.5; // Wheel Radius in mm
    final static double N = 8192; // Ticks per Revolution
    final static double mmPerTick = Math.PI * R / N;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Thread evanControl, evanControlB, ianControl, ianControlB;


        evanControl = new Thread() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    //Evan's Controls:
                    // Stage 1 Control
                    while (gamepad2.left_stick_y != 0) {
                        robot.slideLift.setPower(gamepad2.left_stick_y);
                    }
                    if (gamepad2.left_stick_y == 0) {
                        robot.slideLift.setPower(0);
                    }

                    if (gamepad2.left_trigger != 0) {
                        robot.leftGrip.setPosition(0); //Closed position
                    }

                    if (gamepad2.right_trigger != 0) {
                        robot.rightGrip.setPosition(1); //Closed position
                    }

                    //Gripper Open/Close
                    if (gamepad2.left_bumper) {
                        robot.leftGrip.setPosition(0.5); //Open position
                    }

                    if (gamepad2.right_bumper) {
                        robot.rightGrip.setPosition(0.6); //Open position
                    }
                }
            }
        };

        ianControl = new Thread() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    while (gamepad1.left_stick_y != 0) {
                        robot.setPower(-gamepad1.left_stick_y, -gamepad1.left_stick_y, -gamepad1.left_stick_y, -gamepad1.left_stick_y);
                    }
                }
            }
        };

        ianControlB = new Thread() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    while (gamepad1.right_stick_x != 0) {
                        robot.setPower(gamepad1.right_stick_x, -gamepad1.right_stick_x, gamepad1.right_stick_x, -gamepad1.right_stick_x);
                    }
                }
            }
        };

        waitForStart();

        evanControl.start();
        ianControl.start();
        ianControlB.start();

        while (opModeIsActive()) {
            while (gamepad1.left_stick_x != 0) { //Strafe L/R
                robot.frontLeft.setPower(gamepad1.left_stick_x);
                robot.frontRight.setPower(-gamepad1.left_stick_x);
                robot.backLeft.setPower(-gamepad1.left_stick_x);
                robot.backRight.setPower(gamepad1.left_stick_x);
            }


            double deltaHeading = mmPerTick * (robot.rightEncoder.getCurrentPosition() - robot.leftEncoder.getCurrentPosition()) / L;
            double calculatedHeading = Math.toDegrees(deltaHeading);

            //driver hub info display
            telemetry.addData("IMU Heading", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("Calculated Heading", calculatedHeading);
            telemetry.addLine("\n");
            telemetry.addData("Left Encoder", robot.leftEncoder.getCurrentPosition());
            telemetry.addData("Right Encoder", robot.rightEncoder.getCurrentPosition());
            telemetry.addData("Central Encoder", robot.centralEncoder.getCurrentPosition());
            telemetry.addLine("\n");
            telemetry.addData("Left MM", robot.ticksToMM(robot.leftEncoder.getCurrentPosition()) + " mm");
            telemetry.addData("Right MM", robot.ticksToMM(robot.rightEncoder.getCurrentPosition()) + " mm");
            telemetry.addData("Central MM", robot.ticksToMM(robot.centralEncoder.getCurrentPosition()) + " mm");
            telemetry.addLine("\n");

            telemetry.update();

            robot.stopMotors();
        }
    }
}
