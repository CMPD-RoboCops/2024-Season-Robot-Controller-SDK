package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="LegalTeleOp", group="Linear OpMode")
public class LegalTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor BackLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackRight = null;
    private DcMotor Elevator = null;
    private DcMotor ArmRotate = null;
    private DcMotor ElevatorRotate = null;
    private DcMotor Hook = null;
    private CRServo Claw;

    private boolean limitEnabled = true; // Tracks if the limit is active
    private boolean elevatorRetracted = true; // Tracks if the elevator is fully retracted
    private double elevatorMaxExtensionTime = 2.0; // Maximum extension time for the elevator
    private ElapsedTime elevatorTimer = new ElapsedTime(); // Timer for elevator extension

    // Define arm rotation limits
    private int armInitialPosition = 0; // To store the initial position when the OpMode starts
    private final int armTicksFor20Degrees = 80; // Replace with the actual calculated value

    @Override
    public void runOpMode() {
        // Initialize hardware
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        Elevator = hardwareMap.get(DcMotor.class, "Elevator");
        ArmRotate = hardwareMap.get(DcMotor.class, "ArmRotate");
        ElevatorRotate = hardwareMap.get(DcMotor.class, "ElevatorRotate");
        Hook = hardwareMap.get(DcMotor.class, "Hook");
        Claw = hardwareMap.get(CRServo.class, "Claw");

        // Reverse necessary motors
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        Elevator.setDirection(DcMotor.Direction.REVERSE);

        // Get initial position of ArmRotate
        armInitialPosition = ArmRotate.getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Driving controls
            double axial = -gamepad1.left_stick_y;  // Forward/backward
            double lateral = gamepad1.left_stick_x; // Left/right strafing
            double yaw = gamepad1.right_stick_x;    // Rotation

            // Compute motor powers
            double FrontLeftPower = axial + lateral + yaw;
            double FrontRightPower = axial - lateral - yaw;
            double BackLeftPower = axial - lateral + yaw;
            double BackRightPower = axial + lateral - yaw;

            // Normalize the powers so no value exceeds 1.0
            double maxPower = Math.max(Math.abs(FrontLeftPower), Math.abs(FrontRightPower));
            maxPower = Math.max(maxPower, Math.abs(BackLeftPower));
            maxPower = Math.max(maxPower, Math.abs(BackRightPower));
            if (maxPower > 1.0) {
                FrontLeftPower /= maxPower;
                FrontRightPower /= maxPower;
                BackLeftPower /= maxPower;
                BackRightPower /= maxPower;
            }

            // Set motor powers
            FrontLeft.setPower(FrontLeftPower);
            FrontRight.setPower(FrontRightPower);
            BackLeft.setPower(BackLeftPower);
            BackRight.setPower(BackRightPower);

            // Elevator and elevatorRotate logic
            double elevatorRotatePower = 0;
            if (gamepad1.dpad_up)
                elevatorRotatePower = 0.5;
            else if (gamepad1.dpad_down)
                elevatorRotatePower = -0.5;

            // Update limit status
            if (limitEnabled && Math.abs(ElevatorRotate.getCurrentPosition()) > 60) {
                limitEnabled = false; // Disable limit when elevatorRotate exceeds 60 degrees
            }

            // Elevator Extension Control
            if (gamepad1.left_trigger > 0)
            {
                if (limitEnabled) {
                    if (elevatorTimer.seconds() < elevatorMaxExtensionTime) {
                        Elevator.setPower(0.5);
                    } else {
                        Elevator.setPower(0);
                    }
                } else {
                    Elevator.setPower(0.5); // No limit
                }
                elevatorRetracted = true; // Elevator is extended
            } else if (gamepad1.right_trigger > 0) {
                Elevator.setPower(-0.5); // Retract elevator
                elevatorRetracted = false; // Mark as retracted
                elevatorTimer.reset(); // Reset timer for future extensions
            } else {
                Elevator.setPower(0);
            }

            // Prevent elevatorRotate from moving below 60 degrees unless elevator is retracted
            if (limitEnabled && elevatorRetracted && Math.abs(ElevatorRotate.getCurrentPosition()) < 60) {
                elevatorRotatePower = 0; // Prevent movement
            }

            ElevatorRotate.setPower(elevatorRotatePower);

            // Arm rotation control with limits
            int armCurrentPosition = ArmRotate.getCurrentPosition();
            int minPosition = armInitialPosition - armTicksFor20Degrees;
            int maxPosition = armInitialPosition + armTicksFor20Degrees;

            double armRotatePower = 0;
            if (gamepad1.dpad_right && armCurrentPosition < maxPosition) {
                armRotatePower = 0.5; // Rotate right
            } else if (gamepad1.dpad_left && armCurrentPosition > minPosition) {
                armRotatePower = -0.5; // Rotate left
            }
            ArmRotate.setPower(armRotatePower);

            // Hook control
            if (gamepad1.right_bumper && elevatorRetracted) {
                Hook.setPower(1);
            } else if (gamepad1.left_bumper && elevatorRetracted) {
                Hook.setPower(-1);
            }else
                Hook.setPower(0);

/*
            // Claw control
            if (gamepad1.b) {
                Claw.setPower(-0.5); // Open claw
            } else {
                Claw.setPower(0); // Stop claw
            }
*/
            if (gamepad1.a) {
                Claw.setPower(0);
            } else {
                Claw.setPower(1);
            }

            // Telemetry for debugging
            telemetry.addData("ElevatorRotate Position", ElevatorRotate.getCurrentPosition());
            telemetry.addData("Limit Enabled", limitEnabled);
            telemetry.addData("Elevator Retracted", elevatorRetracted);
            telemetry.addData("Arm Position", armCurrentPosition);
            telemetry.addData("Arm Min/Max", "%d / %d", minPosition, maxPosition);
            telemetry.addData("Hook",Hook);
            telemetry.addData("FrontLeft Power", FrontLeftPower);
            telemetry.addData("FrontRight Power", FrontRightPower);
            telemetry.addData("BackLeft Power", BackLeftPower);
            telemetry.addData("BackRight Power", BackRightPower);
            telemetry.update();
        }
    }
}
