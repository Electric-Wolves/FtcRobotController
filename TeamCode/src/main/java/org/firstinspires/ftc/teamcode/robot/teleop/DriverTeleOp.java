package org.firstinspires.ftc.teamcode.robot.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.JunctionDetection;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "TeleOp", group = "L1")
public class DriverTeleOp extends LinearOpMode {
    double fLX = 0;
    double fRX = 0;
    double bLX = 0;
    double bRX = 0;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);  // Call RobotHardware methods.
        Thread evanControl, evanControlB, ianControl, ianControlB; // Declare threads.

        evanControl = new Thread() { // Thread for lift.
            @Override
            public void run() {
                while (opModeIsActive()) {
                    while (gamepad2.left_stick_y != 0) {
                        robot.slideLift.setPower(-gamepad2.left_stick_y);
                    }

                    robot.slideLift.setPower(0);
                }
            }
        };

        evanControlB = new Thread() { // Thread for gripper.
            @Override
            public void run() {
                while (opModeIsActive()) {
                    //Gripper Open/Close
                    if (gamepad2.left_bumper) { // If left bumper trigger is active
                        robot.openGripper(); //Open Gripper
                    }

                    if (gamepad2.right_bumper) { // If right bumper is active
                        robot.closeGripper(); //Close Gripper
                    }
                }
            }
        };

        ianControl = new Thread() { // Thread for fwd/bwd.
            @Override
            public void run() {
                while (opModeIsActive()) {
                    while (gamepad1.left_stick_y != 0) {
                        fLX = gamepad1.right_stick_x;
                        fRX = -gamepad1.right_stick_x;
                        bLX = gamepad1.right_stick_x;
                        bRX = -gamepad1.right_stick_x;

                        robot.setPower(
                            -gamepad1.left_stick_y + fLX,
                            -gamepad1.left_stick_y + fRX,
                            -gamepad1.left_stick_y + bLX,
                            -gamepad1.left_stick_y + bRX
                        );
                    }

                    while (gamepad1.left_stick_x != 0) {
                        robot.setPower(gamepad1.left_stick_x, -gamepad1.left_stick_x, -gamepad1.left_stick_x, gamepad1.left_stick_x);
                    }
                }
            }
        };

        ianControlB = new Thread() { // Thread for turning
            @Override
            public void run() {
                while (opModeIsActive()) {
                    while (gamepad1.right_stick_x != 0 && gamepad1.left_stick_y == 0) {
                        robot.setPower(
                                gamepad1.right_stick_x,
                                -gamepad1.right_stick_x,
                                gamepad1.right_stick_x,
                                -gamepad1.right_stick_x
                        );
                    }
                }
            }
        };

        telemetry.addLine("Robot has been initialized.\n\nPress > to start TeleOp.");
        telemetry.update();

        waitForStart(); // waits for user to initialize program.

        //Start the threads.
        evanControl.start();
        evanControlB.start();
        ianControl.start();
        ianControlB.start();

        telemetry.setMsTransmissionInterval(1000);

        while (opModeIsActive()) {
            if (gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.left_stick_x == 0) {
                robot.stopMotors();
            }

            // Driver hub info display
            telemetry.addLine(String.format("Left Distance: %.2f mm", lpFilter(robot.getLeftMM())));
            telemetry.addLine(String.format("Center Distance: %.2f mm", lpFilter(robot.getCenterDist())));
            telemetry.addLine("\n");
            telemetry.addLine(String.format("IMU: %.2f Degrees", robot.getHeading())); // Displays control hub's IMU heading.
            telemetry.addLine("\n");
            telemetry.addLine(String.format("Left Encoder: %.2f mm", robot.getLeftMM()));
            telemetry.addLine(String.format("Right Encoder: %.2f mm", robot.getRightMM()));
            telemetry.addLine(String.format("Lift Encoder: %.2f mm", robot.getLiftMM()));
            telemetry.addLine("\n");
            telemetry.addLine(String.format("Right Distance: %.2f mm", robot.getCenterDist()));
            telemetry.addLine("\n");
            telemetry.addData("Red", robot.colorSensor.red());
            telemetry.addData("Blue", robot.colorSensor.blue());
            telemetry.update(); // Update telemetry.
        }
    }

    public double lpFilter(double rawDist) {
        return ((rawDist * 2) - 1);
    }
}
