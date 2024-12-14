package org.firstinspires.ftc.teamcode.processors;

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

import android.graphics.Color;
import android.graphics.Canvas;
import android.graphics.Paint;

import java.util.ArrayList;
import java.util.List;

// TODO: Add method to get coordinates
public class ScoringElementProcessor implements VisionProcessor {

    // Define upper and lower bounds of color ranges.
    private static final Scalar RED_LOWER_BOUND_A = new Scalar(0, 100, 45);
    private static final Scalar RED_UPPER_BOUND_A = new Scalar(10, 255, 255);
    private static final Scalar RED_LOWER_BOUND_B = new Scalar(170, 160, 50);
    private static final Scalar RED_UPPER_BOUND_B = new Scalar(180, 255, 255);
    private static final Scalar YELLOW_LOWER_BOUND = new Scalar(15, 150, 60);
    private static final Scalar YELLOW_UPPER_BOUND = new Scalar(35, 255, 255);
    private static final Scalar BLUE_LOWER_BOUND = new Scalar(95, 80, 0);
    private static final Scalar BLUE_UPPER_BOUND = new Scalar(120, 255, 255);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    /*
     * Finds regions of the image matching the colors, then returns an ArrayList of them.
     * OpenCV uses HSV ranges of 0-180, 0-255, 0-255 instead of the usual 0-360, 0-100, 0-100
     *
     * To test ranges with FTC Dashboard:
     * Core.inRange(hsvImage, new Scalar(RobotConstants.LOW_H, RobotConstants.LOW_S, RobotConstants.LOW_V), new Scalar(RobotConstants.HIGH_H, RobotConstants.HIGH_S, RobotConstants.HIGH_V), mask);
     */
    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {

        // Blur to remove artifacts, resize for performance, and convert to HSV
        Mat hsvImage = new Mat();
        Mat blurredImage = new Mat();
        Mat resizedImage = new Mat();

        Imgproc.blur(input, blurredImage, new Size(11, 11));
        Imgproc.resize(blurredImage, resizedImage, new Size(80, 60));
        Imgproc.cvtColor(resizedImage, hsvImage, Imgproc.COLOR_RGB2HSV);

        // Temporary Mats due to output method of inRange function
        List<Mat> masks = new ArrayList<>();

        Mat redMask = new Mat();
        Mat yellowMask = new Mat();
        Mat blueMask = new Mat();

        // RED
        // Two ranges are needed due to red being at the upper and lower end of the hue scale in HSV
        Mat tempMatA = new Mat();
        Mat tempMatB = new Mat();
        Core.inRange(hsvImage, RED_LOWER_BOUND_A, RED_UPPER_BOUND_A, tempMatA);
        Core.inRange(hsvImage, RED_LOWER_BOUND_B, RED_UPPER_BOUND_B, tempMatB);
        Core.bitwise_or(tempMatA, tempMatB, redMask);
        masks.add(redMask);

        // YELLOW
        Core.inRange(hsvImage, YELLOW_LOWER_BOUND, YELLOW_UPPER_BOUND, yellowMask);
        masks.add(yellowMask);

        // BLUE
        Core.inRange(hsvImage, BLUE_LOWER_BOUND, BLUE_UPPER_BOUND, blueMask);
        masks.add(blueMask);

        return masks;
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
        List<Paint> paints = new ArrayList<>();

        Paint redPaint = new Paint();
        redPaint.setColor(Color.RED);
        redPaint.setStyle(Paint.Style.STROKE);
        redPaint.setStrokeWidth(scaleCanvasDensity * 4);
        paints.add(redPaint);

        Paint yellowPaint = new Paint();
        yellowPaint.setColor(Color.YELLOW);
        yellowPaint.setStyle(Paint.Style.STROKE);
        yellowPaint.setStrokeWidth(scaleCanvasDensity * 4);
        paints.add(yellowPaint);

        Paint bluePaint = new Paint();
        bluePaint.setColor(Color.BLUE);
        bluePaint.setStyle(Paint.Style.STROKE);
        bluePaint.setStrokeWidth(scaleCanvasDensity * 4);
        paints.add(bluePaint);

        // Output of processFrame
        List<Mat> imgMasks = (ArrayList<Mat>) userContext;
        // Stores regions of images
        List<MatOfPoint> regionList = new ArrayList<>();

        // Unused but needed for findContours:
        Mat hierarchy = new Mat();

        // Iterate through colors
        for(int i = 0; i < 3; i++) {

            // Find the largest region of the image matching the color
            Imgproc.findContours(imgMasks.get(i), regionList, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            int matchedRegion = biggestContours(regionList);

            if (!regionList.isEmpty() && Imgproc.contourArea(regionList.get(matchedRegion)) > 50) {

                // Draw outlines
                // Coordinates are multiplied by 12 to account for scaled down image
                Point[] points = findRectPoints(regionList, matchedRegion);
                for (int j = 0; j < points.length; j++) {
                    canvas.drawLine((float)points[j].x * 12, (float)points[j].y * 12, (float)points[(j + 1) % 4].x * 12, (float)points[(j + 1) % 4].y * 12, paints.get(i));
                }

                // Draw centers
                // 1e-5 is used to avoid division by 0
                Moments mu = Imgproc.moments(regionList.get(matchedRegion));
                canvas.drawCircle((float)(mu.m10 * 12 / mu.m00 + 1e-5), (float)(mu.m01 * 12 / mu.m00 + 1e-5), 1, paints.get(i));
            }

            regionList.clear();
        }
    }
}