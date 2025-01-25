package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOpGTB", group="Linear OpMode")
//@Disabled
public class MotorTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor BackLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackRight = null;
    private DcMotor Elevator = null;
    private DcMotor ArmRotate = null;
    private DcMotor ElevatorRotate = null;
    private DcMotor Hook = null;
    private CRServo myServo;

    @Override
    public void runOpMode() {

        //Hardware Mapping
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        Elevator = hardwareMap.get(DcMotor.class, "Elevator");
        ArmRotate = hardwareMap.get(DcMotor.class, "ArmRotate");
        ElevatorRotate = hardwareMap.get(DcMotor.class, "ElevatorRotate");
        Hook = hardwareMap.get(DcMotor.class, "Hook");
        myServo = hardwareMap.get(CRServo.class, "Claw");

        //Motor Direction
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        Elevator.setDirection(DcMotor.Direction.REVERSE);
        ArmRotate.setDirection(DcMotor.Direction.FORWARD);
        ElevatorRotate.setDirection(DcMotor.Direction.FORWARD);

        //Motor Throttle
        double motorSpeed = 0.5; // Current motor speed
        double minSpeed = -1.0; // Minimum motor speed
        double maxSpeed = 1.0; // Maximum motor speed
        double speedIncrement = 0.1; // Increment for speed adjustment
        double power = 0.0;

        //Servo Initialization
        myServo.setPower(0);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double FrontLeftPower = axial + lateral + yaw * motorSpeed;
            double FrontRightPower = axial - lateral - yaw * motorSpeed;
            double BackLeftPower = axial - lateral + yaw * motorSpeed;
            double BackRightPower = axial + lateral - yaw * motorSpeed;

            if (gamepad1.dpad_up)
                ElevatorRotate.setPower(0.5);
            else if (gamepad1.dpad_down)
                ElevatorRotate.setPower(-0.5);
            else
                ElevatorRotate.setPower(0);

            if (gamepad1.dpad_right)
                ArmRotate.setPower(0.5);
            else if (gamepad1.dpad_left)
                ArmRotate.setPower(-0.5);
            else
                ArmRotate.setPower(0);

            if (gamepad1.right_trigger != 0)
                Elevator.setPower(0.5);
            else if (gamepad1.left_trigger != 0)
                Elevator.setPower(-0.5);
            else
                Elevator.setPower(0);

            if (gamepad1.left_bumper)
                Hook.setPower(0.5);
            else if (gamepad1.right_bumper)
                Hook.setPower(-0.5);
            else
                Hook.setPower(0);

            if (gamepad1.y) { // Increase motor speed
                if (motorSpeed + speedIncrement <= maxSpeed) {
                    motorSpeed += speedIncrement;
                } else {
                    telemetry.addData("Error", "Cannot increase speed. Maximum limit reached.");
                }

            } else if (gamepad1.x) { // Decrease motor speed
                if (motorSpeed - speedIncrement >= minSpeed) {
                    motorSpeed -= speedIncrement;
                } else {
                    telemetry.addData("Error", "Cannot decrease speed. Minimum limit reached.");
                }
            }

            if (gamepad1.b) {
                myServo.setPower(-1);
            } else {
                myServo.setPower(1);
            }

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(FrontLeftPower), Math.abs(FrontRightPower));
            max = Math.max(max, Math.abs(BackLeftPower));
            max = Math.max(max, Math.abs(BackRightPower));

            if (max > 1.0) {
                FrontLeftPower /= max;
                FrontRightPower /= max;
                BackLeftPower /= max;
                BackRightPower /= max;
            }


            // Send calculated power to wheels
            FrontLeft.setPower(FrontLeftPower);
            FrontRight.setPower(FrontRightPower);
            BackLeft.setPower(BackLeftPower);
            BackRight.setPower(BackRightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FrontLeft/Right", "%4.2f, %4.2f", FrontLeftPower, FrontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", BackLeftPower, BackRightPower);
            telemetry.addData("ElevatorRotation", "%4.2f", ElevatorRotate.getPower());
            telemetry.addData("ElevatorExtension", "%4.2f", Elevator.getPower());
            telemetry.addData("ArmRotation", "%4.2f", ArmRotate.getPower());
            telemetry.addData("Claw Servo Power", myServo.getPower());
            telemetry.addData("Motor Speed",motorSpeed);
            telemetry.update();



        }
    }
}