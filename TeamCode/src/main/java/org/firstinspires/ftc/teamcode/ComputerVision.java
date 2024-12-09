package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name="Computer Vision", group="Linear OpMode")
public class ComputerVision extends OpMode {

    private VisionPortal visionPortal;
    VisionPortal.Builder visionPortalBuilder;
    private OpenCVProcessor openCVProcessor;
    private AprilTagProcessor aprilTagProcessor;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        openCVProcessor = new OpenCVProcessor();
        Position cameraPos = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPos, cameraOrientation)
                .build();
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, openCVProcessor, aprilTagProcessor);
    }

    @Override
    public void init_loop() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();
        for (AprilTagDetection detection : currentDetections) {
            double myX = detection.robotPose.getPosition().x;
            double myY = detection.robotPose.getPosition().y;
            double myZ = detection.robotPose.getPosition().z;

            double myPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
            double myRoll = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
            double myYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

            telemetry.addData("X", myX);
            telemetry.addData("Y", myY);
            telemetry.addData("Z", myZ);

            telemetry.addData("Pitch", myPitch);
            telemetry.addData("Roll", myRoll);
            telemetry.addData("Yaw", myYaw);

            telemetry.update();
            break;
        }
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
    }
}