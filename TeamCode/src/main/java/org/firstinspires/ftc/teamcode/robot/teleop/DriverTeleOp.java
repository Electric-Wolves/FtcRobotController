package org.firstinspires.ftc.teamcode.robot.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp(name = "TeleOp", group = "L1")
public class DriverTeleOp extends LinearOpMode {
    final static double L = 170; // Distance between Encoder 0 and 1
    final static double B = 150; // Distance from midpoint of L to Encoder 2
    final static double R = 37.5; // Wheel Radius in mm
    final static double N = 8192; // Ticks per Revolution
    final static double mmPerTick = Math.PI * R / N;

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
                        if (gamepad2.left_stick_y > 0) { // if going down ...
                            if (robot.slideLiftEncoder.getCurrentPosition() < 2) {
                                robot.slideLift.setPower(0);
                            }
                            else {
                                robot.slideLift.setPower(-gamepad2.left_stick_y);
                            }
                        }
                        else if (gamepad2.left_stick_y < 0) { // if going up ...
                            robot.slideLift.setPower(-gamepad2.left_stick_y);
                        }
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
                        robot.closeGripper(); //Close GrippeR
                    }
                }
            }
        };

        ianControl = new Thread() { // Thread for fwd/bwd.
            @Override
            public void run() {
                while (opModeIsActive()) {
                    while (gamepad1.left_stick_y != 0) {
                        robot.setPower(-gamepad1.left_stick_y, -gamepad1.left_stick_y, -gamepad1.left_stick_y, -gamepad1.left_stick_y);
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
                    while (gamepad1.right_stick_x != 0) {
                        robot.setPower(gamepad1.right_stick_x, -gamepad1.right_stick_x, gamepad1.right_stick_x, -gamepad1.right_stick_x);
                    }
                }
            }
        };

        waitForStart(); // waits for user to initialize program.

        //Start the threads.
        evanControl.start();
        evanControlB.start();
        ianControl.start();
        ianControlB.start();

        telemetry.setMsTransmissionInterval(1000);

        while (opModeIsActive()) {
            if (gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0) {
                robot.stopMotors();
            }

            //driver hub info display
            telemetry.addLine(String.format("IMU: %.2f Degrees", robot.getHeading())); // Displays control hub's IMU heading.
            telemetry.addLine("\n");
            telemetry.addLine(String.format("Left Encoder: %.2f mm", robot.getLeftMM()));
            telemetry.addLine(String.format("Right Encoder: %.2f mm", robot.getRightMM()));
            telemetry.addLine(String.format("Lift Encoder: %.2f mm", robot.getLiftMM()));
            telemetry.addLine("\n");
            telemetry.addLine(String.format("Left Distance: %.2f mm", robot.getFirstDist()));
            telemetry.addLine(String.format("Right Distance: %.2f mm", robot.getCenterDist()));
            telemetry.addLine("\n");
            telemetry.addData("Red", robot.colorSensor.red());
            telemetry.addData("Blue", robot.colorSensor.blue());
            telemetry.update(); // Update telemetry.
        }
    }
}
