package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name="Computer Vision", group="Linear OpMode")
public class ComputerVision extends OpMode {

    private VisionPortal visionPortal;
    private OpenCVProcessor openCVProcessor;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        openCVProcessor = new OpenCVProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, openCVProcessor);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
    }
}