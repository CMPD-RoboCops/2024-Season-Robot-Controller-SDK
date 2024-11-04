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
import org.opencv.core.Size;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.firstinspires.ftc.teamcode.RobotConstants;

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
        Mat mask = new Mat();
        Mat redMask = new Mat();
        Mat redPoints = new Mat();
        Mat blueMask = new Mat();
        Mat bluePoints = new Mat();
        Mat yellowMask = new Mat();
        Mat yellowPoints = new Mat();

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.blur(input, blurredImage, new Size(11, 11));

            // Ignore anything that says to use BGR2HSV because OpenCV uses it by default because for some reason, it appears to use RGB.
            Imgproc.cvtColor(blurredImage, hsvImage, Imgproc.COLOR_RGB2HSV);

            // OpenCV uses HSV ranges of 0-180, 0-255, 0-255 intead of the usual 0-360, 0-100, 0-100
            // Core.inRange(hsvImage, new Scalar(RobotConstants.LOW_H, RobotConstants.LOW_S, RobotConstants.LOW_V), new Scalar(RobotConstants.HIGH_H, RobotConstants.HIGH_S, RobotConstants.HIGH_V), mask);

            // TODO: Optimize so it doesn't crash.
            // TODO: Adjust values, may not detect in all lighting.
            // TODO: Prevent it from detecting colors other than specimens from the background.

            // RED:
            Core.inRange(hsvImage, new Scalar(0, 160, 160), new Scalar(12, 255, 255), redMask);
            Core.findNonZero(redMask, redPoints);
            Rect redRect = Imgproc.boundingRect(redPoints);
            Imgproc.rectangle(input, redRect, new Scalar (255, 0, 0));
/*
            // BLUE:
            Core.inRange(hsvImage, new Scalar(100, 130, 130), new Scalar(125, 255, 255), blueMask);
            Core.findNonZero(blueMask, bluePoints);
            Rect blueRect = Imgproc.boundingRect(bluePoints);
            Imgproc.rectangle(input, blueRect, new Scalar (0, 0, 255));

            // YELLOW:
            Core.inRange(hsvImage, new Scalar(15, 160, 160), new Scalar(32, 255, 255), yellowMask);
            Core.findNonZero(yellowMask, yellowPoints);
            Rect yellowRect = Imgproc.boundingRect(yellowPoints);
            Imgproc.rectangle(input, yellowRect, new Scalar (0, 255, 255));
*/
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
