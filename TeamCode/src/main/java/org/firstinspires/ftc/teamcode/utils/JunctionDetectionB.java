package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class JunctionDetectionB extends OpenCvPipeline {
    Telemetry telemetry;
    public JunctionDetectionB(Telemetry t) {
        telemetry = t;
    }

    Mat mat = new Mat();

    public enum Location {
        LEFT,
        RIGHT,
        NOT_FOUND
    }

    private JunctionDetection.Location location;

    static final Rect CENTER_ROI = new Rect(
        new Point(280, 200),
        new Point(320, 300)
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
        Mat center = mat.submat(CENTER_ROI);

        // Getting sum of Yellow Value in sub-matrix portions
        double centerValue = Core.sumElems(center).val[0] / CENTER_ROI.area() / 255;

        center.release();

        telemetry.addData("Raw Value", (int) Core.sumElems(center).val[0]);
        telemetry.addData("Percentage", Math.round(centerValue * 100) + "%");
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorJunction = new Scalar(255, 0, 0);
        Scalar colorJunctionB = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, CENTER_ROI, location == JunctionDetection.Location.LEFT? colorJunction:colorJunctionB);

        return mat;
    }

    public JunctionDetection.Location getLocation() {
        return location;
    }
}
