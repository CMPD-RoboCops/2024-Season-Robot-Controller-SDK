package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.processors.ScoringElementProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Autonomous Template")
public class AutonomousTemplate extends OpMode {

    private VisionPortal visionPortal;
    VisionPortal.Builder visionPortalBuilder;
    private ScoringElementProcessor scoringElementProcessor;
    private AprilTagProcessor aprilTagProcessor;

    @Override
    public void init() {
        // Create sample/specimen detection rocessor
        scoringElementProcessor = new ScoringElementProcessor();

        // Create AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(
                        new Position(DistanceUnit.INCH, 0, 6.5, 3, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0)
                )
                .build();

        // Create VisionPortal and add processors
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                scoringElementProcessor,
                aprilTagProcessor
        );
    }

    @Override
    public void init_loop() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
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
    public void loop() {}
}
