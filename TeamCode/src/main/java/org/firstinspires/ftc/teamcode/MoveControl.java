package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp test", group="Linear OpMode")

public class MoveControl extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor BackLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackRight = null;

    private DcMotor Elevator = null;
    private DcMotor ArmRotate = null;
    private DcMotor ElevatorRotate = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FrontLeft  = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft  = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        Elevator = hardwareMap.get(DcMotor.class, "Elevator");
        ArmRotate = hardwareMap.get(DcMotor.class, "ArmRotate");
        ElevatorRotate = hardwareMap.get(DcMotor.class, "ElevatorRotate");

        int ElevatorTarget = 0;

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        Elevator.setDirection(DcMotor.Direction.REVERSE);
        ArmRotate.setDirection(DcMotor.Direction.FORWARD);
        ElevatorRotate.setDirection(DcMotor.Direction.FORWARD);

        //Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            if (gamepad1.dpad_up)
                ElevatorRotate.setPower(1);
            else if (gamepad1.dpad_down)
                ElevatorRotate.setPower(-1);
            else
                ElevatorRotate.setPower(0);

            if (gamepad1.dpad_right)
                ArmRotate.setPower(1);
            else if (gamepad1.dpad_left)
                ArmRotate.setPower(-1);
            else
                ArmRotate.setPower(0);

            if ( gamepad1.right_trigger != 0 && ElevatorTarget < 4250 )
                ElevatorTarget += 5;
            else if ( gamepad1.left_trigger != 0 && ElevatorTarget > 1 )
                ElevatorTarget -= 5;
                Elevator.setTargetPosition(ElevatorTarget);
                Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Elevator.setPower(1);


            //if (ElevatorUp && (TARGET_TICK_VALUE > 0)) TARGET_TICK_VALUE++;
            //else if (ElevatorDown && (TARGET_TICK_VALUE > -10)) TARGET_TICK_VALUE--;

            //Elevator.setTargetPosition(TARGET_TICK_VALUE);    //Sets Target Tick Position
            //Elevator.setPower(1);           //Sets Motor to go to position at 1 power.
            //telemetry.addData("Tick", TARGET_TICK_VALUE);

            // Send calculated power to wheels
            FrontLeft.setPower(leftFrontPower);
            FrontRight.setPower(rightFrontPower);
            BackLeft.setPower(leftBackPower);
            BackRight.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }}
