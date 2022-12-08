package org.firstinspires.ftc.teamcode.robot.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.JunctionDetection;
import org.firstinspires.ftc.teamcode.utils.JunctionDetectionB;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.SignalPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Ian Ultra Auto", group = "Testing")
public class IanUltraAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        SignalPipeline signalPipeline = new SignalPipeline(0.166, 578.272, 578.272, 402.145, 221.506);
        JunctionDetectionB junctionDetection = new JunctionDetectionB(telemetry);

        String tagDetected = "NONE";

        robot.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robot.camera.setPipeline(signalPipeline);
                robot.camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode); // In case of error, we would know something went wrong
                telemetry.update(); //update display
            }
        });

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = signalPipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == 16) {
                        tagDetected = "LEFT";
                        break;
                    }
                    else if (tag.id == 19) {
                        tagDetected = "MIDDLE";
                        break;
                    }
                    else if (tag.id == 13) {
                        tagDetected = "RIGHT";
                        break;
                    }
                }

                if (tagDetected.equals("NONE")) {
                    telemetry.addLine("No tag(s) detected.");
                }
                else {
                    telemetry.addLine(String.format("Tag detected: %s", tagDetected));
                }

            }
            else {
                telemetry.addLine("No tag(s) detected.");

                if (tagDetected.equals("NONE")) {
                    telemetry.addLine("(No tags have ever been detected)");
                }
                else {
                    telemetry.addLine("\nLast tag detected was:");
                    telemetry.addLine(tagDetected);
                }
            }
            telemetry.update();
        }
        robot.camera.stopStreaming();

        robot.closeGripper();

        robot.raiseLift(145, 1);

        // Drive Forward
        robot.move(615, robot.getHeading(), 0.5);

        // Turn Left with imu
        robot.turnToAngle(85, 0.4);

        robot.move(130, 85, 0.4); // 280

        wait(350);

        robot.reverse(70, 85, -0.4); // 220

        while (robot.getHeading() > 35) {
            robot.tankRight(0.4);
        }
        robot.stopMotors();

        robot.move(55, 43, 0.4);

        while (lpFilter(robot.getCenterDist()) >= 300) {
            robot.tankLeft(0.2);
        }
        robot.stopMotors();

        robot.raiseLift(605, 1);

        robot.move(10, robot.getHeading(), 0.2);

        robot.lowerLift(500, -0.3);

        robot.openGripper();

        robot.lowerLift(500, -0.4);

        robot.openGripper();

        robot.reverse(30, robot.getHeading(), -0.3);

        robot.lowerLift(300, -1);

        robot.reverse(60, robot.getHeading(), -0.4);

        while (robot.getHeading() > -85) {
            robot.tankRight(0.3);
        }
        robot.stopMotors();

        robot.lowerLift(100, -0.4);

        robot.move(310, -85, 0.5);

        while (lpFilter(robot.getCenterDist()) >= 16) {
            // If it is too far right, adjust
            if (robot.getHeading() < -85) {
                robot.setPower(
                        0.4 - 0.05,
                        0.4 + 0.05,
                        0.4 - 0.05,
                        0.4 + 0.05
                );
            }
            // If it is too far left, adjust
            else if (robot.getHeading() > -85) {
                robot.setPower(
                        0.4 + 0.05,
                        0.4 - 0.05,
                        0.4 + 0.05,
                        0.4 - 0.05
                );
            }
            // Maintain otherwise
            else {
                robot.setPower(0.4, 0.4, 0.4, 0.4);

            }
            robot.stopMotors();
        }

    }

    public double lpFilter(double rawDist) {
        return ((rawDist * 2) - 1);
    }

    public void wait(int ms) { // Waiting method
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}
