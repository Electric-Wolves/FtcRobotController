package org.firstinspires.ftc.teamcode.utils;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CameraColorDetection extends OpenCvPipeline {
    Mat YCbCr = new Mat();
    Mat leftCrop;
    Mat rightCrop;

    double leftAvgFinal;
    double rightAvgFinal;

    Mat output = new Mat();

    Scalar rectColor = new Scalar(255, 0, 0);

    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

        Rect leftRect = new Rect(1, 1, 319, 359);
        Rect rightRect = new Rect(320, 1, 319, 359);

        input.copyTo(output);

        Imgproc.rectangle(output, leftRect, rectColor, 2);
        Imgproc.rectangle(output, rightRect, rectColor, 2);

        leftCrop = YCbCr.submat(leftRect);
        rightCrop = YCbCr.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop, 2);
        Core.extractChannel(rightCrop, rightCrop, 2);

        Scalar leftAvg = Core.mean(leftCrop);
        Scalar rightAvg = Core.mean(rightCrop);

        leftAvgFinal = leftAvg.val[0];
        rightAvgFinal = rightAvg.val[0];

        return output;
    }
}
