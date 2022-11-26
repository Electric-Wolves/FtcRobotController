package org.firstinspires.ftc.teamcode.utils;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;

public class RobotHardware {
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    public DcMotorSimple slideLift = null;

    public Servo leftGrip = null;
    public Servo rightGrip = null;

    public DcMotor leftEncoder = null;
    public DcMotor rightEncoder = null;
    public DcMotor slideLiftEncoder = null;

    public TouchSensor touchSensor = null;

    public BNO055IMU imu = null;

    public OpenCvCamera camera = null;

    HardwareMap hardwareMap = null;

    public RobotHardware(HardwareMap hwMap) {
        initialize(hwMap);
    }

    private void initialize(HardwareMap hwMap) {
        hardwareMap = hwMap;

        // Main Drivetrain Motors
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        // SPARK Mini Stage one and two motors
        slideLift = hardwareMap.get(DcMotorSimple.class, "slideLift");

        // Gripper Servos
        leftGrip = hardwareMap.get(Servo.class, "leftGrip");
        rightGrip = hardwareMap.get(Servo.class, "rightGrip");

        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        // Encoders occupy built-in motor encoder ports, so needs to be shadowed
        leftEncoder = frontLeft;
        rightEncoder = frontRight;
        slideLiftEncoder = backLeft;

        // IMU Parameter setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "21301-Cam"), cameraMonitorViewId);

        // Adjusting the Forward (positive value) Direction to be uniform
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // When the motors are not moving, they will brake
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Not using runToPosition for enhanced navigation
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetEncoders();
    }

    public void setPower(double fL, double fR, double bL, double bR) {
        frontLeft.setPower(fL);
        frontRight.setPower(fR);
        backLeft.setPower(bL);
        backRight.setPower(bR);
    }

    // Negative = left, Positive = right
    public void turnToAngle(double targetHeading, double power) {
        if (targetHeading > 0) {
            while (getHeading() < targetHeading) {
                tankLeft(power);
            }
        }
        else {
            while (getHeading() > targetHeading) {
                tankRight(power);
            }
        }
        stopMotors();
    }

    // Method for driving forward/straight while maintaining a constant heading
    public void move(double targetMM, double targetHeading, double power) {
        // Setting the initial starting position of the robot
        double startingTicks = leftEncoder.getCurrentPosition();

        // Repeat until target/goal is met
        while (Math.abs(ticksToMM(leftEncoder.getCurrentPosition() - startingTicks)) < targetMM) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // If it is too far right, adjust
            if (angles.firstAngle < targetHeading) {
                setPower(power - 0.05, power + 0.0, power - 0.05,power + 0.05);
            }
            // If it is too far  left, adjust
            else if (angles.firstAngle > targetHeading) {
                setPower(power + 0.05, power - 0.05, power + 0.05, power - 0.05);
            }
            // Maintain otherwise
            else {
                setPower(power, power, power, power);
            }
        }
        stopMotors();
    }

    // Method for driving forward/straight while maintaining a constant heading
    public void reverse(double targetMM, double targetHeading, double power) {
        // Setting the initial starting position of the robot
        double startingTicks = leftEncoder.getCurrentPosition();

        // Repeat until target/goal is met
        while (Math.abs(ticksToMM(leftEncoder.getCurrentPosition() - startingTicks)) < targetMM) {
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            // If it is too far right, adjust
            if (angles.firstAngle < targetHeading) {
                setPower(power + 0.05, power - 0.0, power + 0.05,power - 0.05);
            }
            // If it is too far  left, adjust
            else if (angles.firstAngle > targetHeading) {
                setPower(power - 0.05, power + 0.05, power - 0.05, power + 0.05);
            }
            // Maintain otherwise
            else {
                setPower(power, power, power, power);
            }
        }
        stopMotors();
    }

    public void tankRight(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void tankLeft(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    public void strafeLeft(double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
    }

    public void strafeRight(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
    }

    public double ticksToMM(double ticks) {
        double mmPerTick = Math.PI * 30 / 8192;
        return ticks * mmPerTick;
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void openGripper() {
        leftGrip.setPosition(0.5);
        rightGrip.setPosition(0.6);
    }

    public void closeGripper() {
        leftGrip.setPosition(0);
        rightGrip.setPosition(1);
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}
