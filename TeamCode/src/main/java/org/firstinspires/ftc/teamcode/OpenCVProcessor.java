package org.firstinspires.ftc.teamcode;

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

public class OpenCVProcessor implements VisionProcessor {

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    Mat hsvImage = new Mat();
    Mat blurredImage = new Mat();
    Mat resizedImage = new Mat();
    Mat[] masks = new Mat[] { new Mat(), new Mat(), new Mat() };

    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();
    Moments mu = new Moments();
    RotatedRect rect = new RotatedRect();
    MatOfPoint2f rectPoints = new MatOfPoint2f();
    Mat box = new Mat();

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // Blur to remove artifacts
        Imgproc.blur(input, blurredImage, new Size(11, 11));

        // Resize for performance
        Imgproc.resize(blurredImage, resizedImage, new Size(80, 60));

        // Convert to HSV
        // NOTE: Ignore anything that says to use BGR2HSV
        Imgproc.cvtColor(resizedImage, hsvImage, Imgproc.COLOR_RGB2HSV);

        /*
         * The following finds the center of the largest contour of the mask.
         * Contour is the name used by OpenCV to describe a region of the image.
         *
         * OpenCV uses HSV ranges of 0-180, 0-255, 0-255 instead of the usual 0-360, 0-100, 0-100
         *
         * 1e-5 is used to avoid division by 0 according to OpenCV docs
         * Multiplied by for to account for scaled down image
         *
         * Test ranges with FTC Dashboard:
         * Core.inRange(hsvImage, new Scalar(RobotConstants.LOW_H, RobotConstants.LOW_S, RobotConstants.LOW_V), new Scalar(RobotConstants.HIGH_H, RobotConstants.HIGH_S, RobotConstants.HIGH_V), mask);
         */

        // RED
        Core.inRange(hsvImage, new Scalar(0, 100, 45), new Scalar(10, 255, 255), masks[1]);
        Core.inRange(hsvImage, new Scalar(170, 160, 50), new Scalar(180, 255, 255), masks[2]);
        Core.bitwise_or(masks[1], masks[2], masks[0]);
        // YELLOW
        Core.inRange(hsvImage, new Scalar(15, 150, 60), new Scalar(35, 255, 255), masks[1]);
        // BLUE
        Core.inRange(hsvImage, new Scalar(95, 80, 0), new Scalar(120, 255, 255), masks[2]);
        return null;
    }

    public Point[] findRectPoints(List<MatOfPoint> contours, int contoursIdx) {
        rectPoints = new MatOfPoint2f( contours.get(contoursIdx).toArray() );
        rect = Imgproc.minAreaRect(rectPoints);
        Imgproc.boxPoints(rect, box);
        Point[] points = new Point[4];
        rect.points(points);
        return points;
    }
    public int biggestContours(List<MatOfPoint> contours) {
        /*
         * Finds the largest contour Id when given a list of contours.
         */
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
        // Set paint color
        Paint paint = new Paint();
        paint.setColor(Color.RED);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(scaleCanvasDensity * 4);

        // Draw stuff
        for(int i = 0; i < 3; i++) {
            contours.clear();
            Imgproc.findContours(masks[i], contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            int Idx = biggestContours(contours);
            if (!contours.isEmpty() && Imgproc.contourArea(contours.get(Idx)) > 50 && contours.size() <= Idx) {
                Point[] points = findRectPoints(contours, Idx);
                for (int j = 0; j < 4; ++j) {
                    canvas.drawLine((float)points[j].x * 4, (float)points[j].y * 4, (float)points[(j + 1) % 4].x * 4, (float)points[(j + 1) % 4].y * 4, paint);
                }
                mu = Imgproc.moments(contours.get(Idx));
                canvas.drawCircle((float)(mu.m10 * 4 / mu.m00 + 1e-5), (float)(mu.m01 * 4 / mu.m00 + 1e-5), 1, paint);
            }
        }
    }
}