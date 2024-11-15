/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Size;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="OpenCV Testing", group="Linear OpMode")
public class TestingOpenCV extends LinearOpMode {

    OpenCvCamera camera;

    @Override
    public void runOpMode() {
        // TODO: Get webcam name
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.setPipeline(new SamplePipeline());

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            sleep(100);
        }
    }
    class SamplePipeline extends OpenCvPipeline
    {
        boolean viewportPaused = false;

        Mat hsvImage = new Mat();
        Mat blurredImage = new Mat();
        Mat resizedImage = new Mat();
        Mat mask = new Mat();
        Mat redMask = new Mat();
        Mat redPoints = new Mat();
        Mat blueMask = new Mat();
        Mat bluePoints = new Mat();
        Mat yellowMask = new Mat();
        Mat yellowPoints = new Mat();

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Moments mu = new Moments();

        public int biggestContours(List<MatOfPoint> contours) {
            /*
             * Finds the largest contour Id when given a list of contours.
             */
            double maxVal = 0;
            int maxValIdx = 0;
            for (int contourIdx = 0; contourIdx < contours.size(); contourIdx++)
            {
                double contourArea = Imgproc.contourArea(contours.get(contourIdx));
                if (maxVal < contourArea)
                {
                    maxVal = contourArea;
                    maxValIdx = contourIdx;
                }
            }
            return maxValIdx;
        }

        @Override
        public Mat processFrame(Mat input)
        {
            // Blur to remove artifacts
            Imgproc.blur(input, blurredImage, new Size(11, 11));
            // Resize for performance
            Imgproc.resize(blurredImage, resizedImage, new Size(50, 50));
            // Convert to HSV
            // NOTE: Ignore anything that says to use BGR2HSV
            Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_RGB2HSV);

            // TODO: Adjust color values.
            // OpenCV uses HSV ranges of 0-180, 0-255, 0-255 instead of the usual 0-360, 0-100, 0-100
            // Core.inRange(hsvImage, new Scalar(RobotConstants.LOW_H, RobotConstants.LOW_S, RobotConstants.LOW_V), new Scalar(RobotConstants.HIGH_H, RobotConstants.HIGH_S, RobotConstants.HIGH_V), mask);

            /*
             * The following finds the center of the largest contour of the mask.
             * Contour is the name used by OpenCV to describe a region of the image.
             */

            // RED
            Core.inRange(hsvImage, new Scalar(0, 100, 50), new Scalar(18, 255, 255), redMask);
            contours.clear();
            Imgproc.findContours(redMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            int redIdx = biggestContours(contours);
            if ( redIdx > 0 && Imgproc.contourArea(contours.get(redIdx)) > 50 ) {
                mu = Imgproc.moments(contours.get(redIdx));
                // 1e-5 is used to avoid division by 0 according to OpenCV docs
                Imgproc.circle(input, new Point(mu.m10 / mu.m00 + 1e-5, mu.m01 / mu.m00 + 1e-5), 4, new Scalar(0, 255, 0), -1);
            }

            // YELLOW
            Core.inRange(hsvImage, new Scalar(15, 100, 50), new Scalar(32, 255, 255), yellowMask);
            contours.clear();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            int yellowIdx = biggestContours(contours);
            if ( yellowIdx > 0 && Imgproc.contourArea(contours.get(yellowIdx)) > 50 ) {
                mu = Imgproc.moments(contours.get(yellowIdx));
                Imgproc.circle(input, new Point(mu.m10 / mu.m00 + 1e-5, mu.m01 / mu.m00 + 1e-5), 4, new Scalar(0, 255, 0), -1);
            }

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                camera.pauseViewport();
            }
            else
            {
                camera.resumeViewport();
            }
        }
    }
}
