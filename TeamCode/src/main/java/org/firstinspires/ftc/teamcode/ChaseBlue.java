package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.processors.ChaseBlueProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.Point;

@Autonomous(name = "Chase Blue")
public class ChaseBlue extends OpMode {

    private DcMotor FrontLeft;
    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor BackRight;

    private VisionPortal visionPortal;
    private ChaseBlueProcessor chaseBlueProcessor;

    @Override
    public void init() {
        // Create sample/specimen detection rocessor
        chaseBlueProcessor = new ChaseBlueProcessor();

        // Create VisionPortal and add processors
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                chaseBlueProcessor
        );

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void start() {
        visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
        Point bluePos = chaseBlueProcessor.getBluePos();
        telemetry.addData("BluePosX", bluePos.x);
        telemetry.addData("BluePosY", bluePos.y);

        FrontRight.setPower(0);
        BackRight.setPower(0);
        FrontLeft.setPower(0);
        BackLeft.setPower(0);

        // TODO: Reverse
        // TODO: New detections while running
        if (bluePos.x < 0.4) {
            FrontLeft.setPower(0.5);
            BackLeft.setPower(0.5);
        } else if (bluePos.x > 0.6) {
            FrontRight.setPower(0.5);
            BackRight.setPower(0.5);
        }
    }
}
