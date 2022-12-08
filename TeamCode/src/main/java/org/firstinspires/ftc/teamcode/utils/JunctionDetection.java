package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class JunctionDetection extends OpenCvPipeline {
    Telemetry telemetry;
    public JunctionDetection(Telemetry t) {
        telemetry = t;
    }

    Mat mat = new Mat();

    public enum Location {
        LEFT,
        RIGHT,
        NOT_FOUND
    }

    private Location location;

    // Left and Right Regions of Interest on Camera Frame
    static final Rect LEFT_ROI = new Rect(
        new Point(100, 200),
        new Point(220, 300)
    );
    static final Rect RIGHT_ROI = new Rect(
        new Point(230, 200),
        new Point(350, 300)
    );

    // 40% of Yellow, will need Calibration
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    @Override
    public Mat processFrame(Mat input) {
        // Convert from RGB to Hue, Saturation, Value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // Upper and Lower limits of Yellow Junction
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);

        // Thresholding for Gray Scale Image
        Core.inRange(mat, lowHSV, highHSV, mat);

        // Creating sub-matrix portions of original image
        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);

        // Getting sum of Yellow Value in sub-matrix portions
        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();

        telemetry.addData("Left Raw Value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right Raw Value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left Percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right Percentage", Math.round(rightValue * 100) + "%");

        boolean junctionLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean junctionRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (junctionLeft && junctionRight) {
            location = Location.NOT_FOUND;
            telemetry.addData("Junction Location", "Not Detected");
        }
        if (junctionLeft) {
            location = Location.RIGHT;
            telemetry.addData("Junction Location", "Right");
        }
        else {
            location = Location.LEFT;
            telemetry.addData("Junction Location", "Left");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorJunction = new Scalar(255, 0, 0);
        Scalar colorJunctionB = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorJunction:colorJunctionB);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorJunction:colorJunctionB);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}
