package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="AutonomousMode", group="Linear OpMode")
public class AutonomousMode extends LinearOpMode {

    private DcMotor FrontLeft = null;
    private DcMotor BackLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackRight = null;
    private DcMotor Elevator = null;
    private DcMotor ArmRotate = null;
    private DcMotor ElevatorRotate = null;
    private CRServo Claw = null;

    private String alliance = "Blue"; // Default to Blue Alliance
    private String startPosition = "Front"; // Default to Front Starting Position
    private boolean skipBasket = false; // Option to skip basket
    private boolean delayedStart = false; // Option for delayed start

    @Override
    public void runOpMode() {

        // Hardware Mapping
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        Elevator = hardwareMap.get(DcMotor.class, "Elevator");
        ArmRotate = hardwareMap.get(DcMotor.class, "ArmRotate");
        ElevatorRotate = hardwareMap.get(DcMotor.class, "ElevatorRotate");
        Claw = hardwareMap.get(CRServo.class, "Claw");

        // Motor Direction
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        ArmRotate.setDirection(DcMotor.Direction.FORWARD);
        ElevatorRotate.setDirection(DcMotor.Direction.FORWARD);

        // Servo Initialization: Start with claw open
        Claw.setPower(1);

        // Allow driver to choose alliance, position, skipBasket, and delayedStart
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_left) {
                alliance = "Blue";
            } else if (gamepad1.dpad_right) {
                alliance = "Red";
            }

            if (gamepad1.dpad_up) {
                startPosition = "Front";
            } else if (gamepad1.dpad_down) {
                startPosition = "Back";
            }

            if (gamepad1.a) {
                skipBasket = true;
            } else if (gamepad1.b) {
                skipBasket = false;
            }

            if (gamepad1.x) {
                delayedStart = true;
            } else if (gamepad1.y) {
                delayedStart = false;
            }

            telemetry.addData("Alliance", "Blue (D-Pad Left), Red (D-Pad Right): %s", alliance);
            telemetry.addData("Starting Position", "Front (D-Pad Up), Back (D-Pad Down): %s", startPosition);
            telemetry.addData("Skip Basket", "Yes (A), No (B): %s", skipBasket ? "Yes" : "No");
            telemetry.addData("Delayed Start", "Yes (X), No (Y): %s", delayedStart ? "Yes" : "No");
            telemetry.update();
        }

        // Close claw to grab sample
        Claw.setPower(-1);
        sleep(500); // Allow time for the claw to close
        Claw.setPower(0);

        waitForStart();

        if (opModeIsActive()) {

            // Handle delayed start
            if (delayedStart) {
                sleep(5000); // 5-second delay
            }

            // Go to basket based on alliance, unless skipping basket
            if (!skipBasket) {
                if (alliance.equals("Blue")) {
                    if (startPosition.equals("Front")) {
                        driveForward(18);  // Move forward 18 inches
                        strafeRight(24);  // Turn Right
                    } else {
                        driveForward(18);  // Move forward 18 inches
                        strafeLeft(24);  // Turn Left
                    }
                } else if (alliance.equals("Red")) {
                    if (startPosition.equals("Front")) {
                        driveForward(18);  // Move forward 18 inches
                        strafeLeft(24);  // Turn Left
                    } else {
                        driveForward(18);  // Move forward 18 inches
                        strafeRight(24);  // Turn Right
                    }
                }

                // Raise Elevator Arm
                ElevatorRotate.setPower(0.5);
                sleep(1000); // Adjust based on height needed to successfully reach top basket
                ElevatorRotate.setPower(0);

                // Extend elevator
                Elevator.setPower(0.5);
                sleep(2000); // Adjust based on elevator travel time
                Elevator.setPower(0);

                // Drop sample
                Claw.setPower(1);
                sleep(500); // Open claw
                Claw.setPower(0);

                // Retract elevator
                Elevator.setPower(-0.5);
                sleep(2000); // Adjust based on elevator travel time
                Elevator.setPower(0);

                // Perform specific parking maneuver after basket
                performPostBasketParking(alliance);
            } else {
                // Perform parking maneuver directly if skipping basket
                performSkipBasketParking(startPosition);
            }

            telemetry.addData("Status", "Autonomous Complete");
            telemetry.update();
        }
    }

    private void performPostBasketParking(String alliance) {
        // Execute 180-degree turn and park based on alliance
        turn(180); // 180-degree turn
        driveForward(132); // Drive 11 feet

        if (alliance.equals("Blue")) {
            turn(90); // Turn right
            driveForward(6); // Drive forward 6 inches
        } else if (alliance.equals("Red")) {
            turn(-90); // Turn left
            driveForward(6); // Drive forward 6 inches
        }
    }

    private void performSkipBasketParking(String startPosition) {
        // Park directly if skipping basket
        if (startPosition.equals("Front")) {
            turnToFieldFront();
            driveForward(36); // Drive forward 3 feet
        } else {
            turnToFieldFront();
            driveForward(72); // Drive forward 6 feet
        }
    }

    private void turn(int degrees) {
        // Placeholder for turning logic based on degrees
        telemetry.addData("Turning", "%d degrees", degrees);
        telemetry.update();
        sleep(Math.abs(degrees) * 10); // Simulate turn time
    }

    private void turnToFieldFront() {
        // Placeholder for turning towards the front of the field
        telemetry.addData("Turning", "To field front");
        telemetry.update();
        sleep(500); // Simulate turn time
    }

    private void driveForward(int distance) {
        // Logic to drive forward a certain distance (implement encoder-based driving)
        FrontLeft.setPower(0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(0.5);
        telemetry.addData("Driving Forward", "Distance: %d inches", distance);
        telemetry.update();
        sleep(distance * 50); // Placeholder conversion factor for time
        stopMotors();
    }

    private void strafeRight(int distance) {
        // Logic to strafe right a certain distance
        FrontLeft.setPower(0.5);
        BackLeft.setPower(-0.5);
        FrontRight.setPower(-0.5);
        BackRight.setPower(0.5);
        telemetry.addData("Strafing Right", "Distance: %d inches", distance);
        telemetry.update();
        sleep(distance * 50); // Placeholder conversion factor for time
        stopMotors();
    }

    private void strafeLeft(int distance) {
        // Logic to strafe left a certain distance
        FrontLeft.setPower(-0.5);
        BackLeft.setPower(0.5);
        FrontRight.setPower(0.5);
        BackRight.setPower(-0.5);
        telemetry.addData("Strafing Left", "Distance: %d inches", distance);
        telemetry.update();
        sleep(distance * 50); // Placeholder conversion factor for time
        stopMotors();
    }

    private void stopMotors() {
        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
        telemetry.addData("Motors Stopped", "All motors set to zero");
        telemetry.update();
    }
}
