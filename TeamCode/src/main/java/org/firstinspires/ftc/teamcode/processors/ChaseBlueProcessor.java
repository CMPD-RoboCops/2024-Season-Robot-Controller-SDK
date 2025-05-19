package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

// TODO: Add method to get coordinates
public class ChaseBlueProcessor implements VisionProcessor {

    // Define upper and lower bounds of color ranges.
    private static final Scalar BLUE_LOWER_BOUND = new Scalar(95, 80, 0);
    private static final Scalar BLUE_UPPER_BOUND = new Scalar(120, 255, 255);

    private Point bluePos = new Point();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    /*
     * Finds regions of the image matching the colors, then returns an ArrayList of them.
     * OpenCV uses HSV ranges of 0-180, 0-255, 0-255 instead of the usual 0-360, 0-100, 0-100
     *
     * To test ranges with FTC Dashboard:
     * Core.inRange(hsvImage, new Scalar(RobotConstants.LOW_H, RobotConstants.LOW_S, RobotConstants.LOW_V), new Scalar(RobotConstants.HIGH_H, RobotConstants.HIGH_S, RobotConstants.HIGH_V), mask);
     */

    public Point getBluePos() {
        return bluePos;
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        // Blur to remove artifacts, resize for performance, and convert to HSV
        Mat hsvImage = new Mat();
        Mat blurredImage = new Mat();
        Mat resizedImage = new Mat();

        Imgproc.blur(input, blurredImage, new Size(11, 11));
        Imgproc.resize(blurredImage, resizedImage, new Size(80, 60));
        Imgproc.cvtColor(resizedImage, hsvImage, Imgproc.COLOR_RGB2HSV);

        Mat blueMask = new Mat();
        Core.inRange(hsvImage, BLUE_LOWER_BOUND, BLUE_UPPER_BOUND, blueMask);

        return blueMask;
    }

    /*
     * Finds corners of a rectangle bounding a contour and returns an array
     */
    public Point[] findRectPoints(List<MatOfPoint> regions, int matchedRegionId) {

        // Create array of points from matched region
        MatOfPoint2f regionPoints = new MatOfPoint2f( regions.get(matchedRegionId).toArray() );

        // Create OpenCV rectangle when given
        RotatedRect rect = Imgproc.minAreaRect(regionPoints);

        // Convert vertices to array
        Point[] points = new Point[4];
        rect.points(points);

        return points;
    }

    /*
     * Finds the largest contour id when given a list of contours.
     */
    public int biggestContours(List<MatOfPoint> contours) {

        double maxVal = 0;
        int maxValIdx = 0;

        for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
            double contourArea = Imgproc.contourArea(contours.get(contourIdx));

            if (maxVal < contourArea) {
                maxVal = contourArea;
                maxValIdx = contourIdx;
            }
        }

        return maxValIdx;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        // Create list of paints
        Paint bluePaint = new Paint();
        bluePaint.setColor(Color.BLUE);
        bluePaint.setStyle(Paint.Style.STROKE);
        bluePaint.setStrokeWidth(scaleCanvasDensity * 4);

        // Output of processFrame
        Mat imgMask = (Mat) userContext;
        // Stores regions of images
        List<MatOfPoint> regionList = new ArrayList<>();
        // Unused but needed for findContours:
        Mat hierarchy = new Mat();

        // Find the largest region of the image matching the color
        Imgproc.findContours(imgMask, regionList, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        int matchedRegion = biggestContours(regionList);

        if (!regionList.isEmpty() && Imgproc.contourArea(regionList.get(matchedRegion)) > 50) {

            // Draw outlines
            // Coordinates are multiplied by 12 to account for scaled down image
            Point[] points = findRectPoints(regionList, matchedRegion);

            // Draw centers
            // 1e-5 is used to avoid division by 0
            Moments mu = Imgproc.moments(regionList.get(matchedRegion));
            canvas.drawCircle((float)(mu.m10 * 12 / mu.m00 + 1e-5), (float)(mu.m01 * 12 / mu.m00 + 1e-5), 1, bluePaint);
            bluePos.x = (float)((mu.m10 / 80) / (mu.m00 + 1e-5));
            bluePos.y = (float)((mu.m01 / 60) / (mu.m00 + 1e-5));
        }

        regionList.clear();
    }
}