package org.firstinspires.ftc.teamcode.robot.auto;

/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.SignalPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Right Autonomous", group = "Testing")
public class RightAuto extends LinearOpMode {
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagSize = 0.166;

    // Tag ids for Parking Positions from 36h11 april tags
    int LEFT = 16;
    int MIDDLE = 19;
    int RIGHT = 13;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        SignalPipeline signalPipeline = new SignalPipeline(tagSize, fx, fy, cx, cy);

        Thread slideLiftRaise, slideLiftLower, secondLiftRaise;

        // Pre-Load Drop
        slideLiftRaise = new Thread() {
            @Override
            public void run() {
                robot.raiseLift(605, 0.85); // Raise the lift up to drop height

                robot.move(45, robot.getHeading(), 0.2); // Drive up to the junction

                robot.lowerLift(500, -0.4); // Lower the lift partially such that the cone is on the junction

                robot.openGripper(); // Open the gripper to drop the cone

                robot.lowerLift(100, -0.4); // Lower the lift the remaining distance
            }
        };

        /* slideLiftLower = new Thread() {
            @Override
            public void run() {
                robot.lowerLift(375, -0.65);
            }
        };

        secondLiftRaise = new Thread() {
            @Override
            public void run() {
                robot.raiseLift(605, 0.85);

                robot.move(125, 41, 0.4);

                robot.lowerLift(500, -0.3);

                robot.openGripper();
            }
        };*/

        // Lines 100 -175: Camera detection
        AprilTagDetection tagOfInterest = null;

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

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = signalPipeline.getLatestDetections();

            robot.closeGripper();

            if(currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

            telemetry.update();
            sleep(20);
        }

        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        double relativeOrigin = robot.getHeading(); // Set the robot's current heading at 0

        robot.move(800, relativeOrigin, 0.6); // Drive forward to push signal cone out of the robot's path

        wait(100); // Pause slightly

        slideLiftRaise.start(); // Begin raising the lift in preparation for the pre-load cone

        robot.reverse(182, relativeOrigin, -0.3); // Drive in reverse to prepare for the pre-load drop

        // Turn to face the junction (187 - 189)
        while (robot.getHeading() < 30) {
            robot.tankLeft(0.4);
        }
        robot.stopMotors(); // Stop

        robot.move(85, robot.getHeading(), 0.4); // Drive up to the junction

        wait(9000); // Wait for the thread to finish

        robot.reverse(35, 30, -0.5);

        robot.turnToAngle(65, 0.5);

        robot.reverse(65, 30, -0.5); // Drive away from the junction

      /*  slideLiftLower.start();

        robot.reverse(100, robot.getHeading(), -0.5);

        robot.turnToAngle(-85, 0.4);

        robot.move(225, -85, 0.5);

        robot.closeGripper();

        robot.raiseLift(200, 0.5);

        secondLiftRaise.start();

        robot.reverse(275, -85, -0.5);

        robot.turnToAngle(41, 0.5);
*/

        // Parking for positions: 2 & 5
        if (tagOfInterest == null || tagOfInterest.id == MIDDLE) {
            robot.turnToAngle(-179, 0.3); // Turn to face the Driver Box
            robot.move(115, -170, 0.4); // Drive to the middle of the square
        }

        // Parking for position: 1 & 4
        else if (tagOfInterest.id == LEFT) {
            robot.turnToAngle(85, 0.5); // Turn to face 90 degrees to the left of the Driver Box
            robot.move(300, 85, 0.5); // Drive 1 square
            robot.turnToAngle(-170, 0.5); // Turn to face the Driver Box
            robot.move(150, -170, 0.5); // Drive to the middle of the square
        }

        // Parking for positions: 3 & 6
        else if (tagOfInterest.id == RIGHT) {
            robot.turnToAngle(-85, 0.5); // Turn to face 90 degrees to the right of the Drive Box
            robot.move(300, 0.85, 0.5); // Drive 1 square
            robot.turnToAngle(-85, 0.5);
        }
        robot.stopMotors(); // Stop moving
    }

    // More camera code
    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    public void wait(int ms) { // Waiting method
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}
