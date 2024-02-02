package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class PixelRecognizerNew extends OpenCvPipeline {
    private int shippingHubLevel = 2;
    double leftValue;
    double rightValue;

    double centerX;
    int finalCenterX;
    double centerY;
    int finalCenterY;
    int values;

    public int getPixelFieldPos(int[] init, int[] run) {
        if (Math.sqrt(Math.pow((init[0] - run[0]), 2) + Math.pow((init[1] - run[1]), 2)) > 10) {
            if (init[0] > run[0]) {
                shippingHubLevel = 1;
            }
            else if (init[0] < run[0]) {
                shippingHubLevel = 3;
            }
            else {
                shippingHubLevel = 2;
            }
        }
        return shippingHubLevel;
    }

    public double getLeftValue() {
        return leftValue;
    }

    public double getRightValue() {
        return rightValue;
    }

    static final Rect BoundingBox = new Rect(
            new Point(0, 20),
            new Point(320, 110)
    );

    public int getPixelPosX() {
        return finalCenterX;
    }

    public int getPixelPosY() {
        return finalCenterY;
    }

    // Recognizes the shipping hub level based on where the team shipping element is located
    // Create two possible boxes it can be in

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Scalar lowHSV = new Scalar(110, 100, 35); // purple lower in hsv
        Scalar highHSV = new Scalar(150, 255, 255); // purple upper in hsv
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV); // convert to hsv
        Core.inRange(input, lowHSV, highHSV, mat); // make purple white, everything else black
        Mat bound = mat.submat(BoundingBox);
        for (int y = 0; y < bound.rows(); y++) {
            for (int x = 0; x < bound.cols(); x++) {
                double[] data = bound.get(y, x); // Get the pixel value at (y, x)
                if (data[0] == 255) { // Check if the pixel is white
                    // Print the coordinates of the white pixel
                    values++;
                    centerX += x + BoundingBox.x;
                    centerY += y + BoundingBox.y;
                }
            }
        }

        finalCenterX = (int)(centerX / values);
        finalCenterY = (int)(centerY / values);

        Imgproc.rectangle(mat, BoundingBox, new Scalar(255, 0, 0), 2); // draw rectangle around the bounding area

        if (values > 0) {
            Imgproc.circle(mat, new Point(finalCenterX, finalCenterY), 5, new Scalar(110, 100, 25), -1);
        }

        bound.release();
        return mat;
    }


}