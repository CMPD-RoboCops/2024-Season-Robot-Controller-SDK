package org.firstinspires.ftc.teamcode;/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

@TeleOp(name="TeleOpGTBElevatorEncoders", group="Linear OpMode")
public class MotorTestElevatorEncoders extends LinearOpMode {

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
    private CRServo Claw;

    @Override
    public void runOpMode() {

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        Elevator = hardwareMap.get(DcMotor.class, "Elevator");
        ArmRotate = hardwareMap.get(DcMotor.class, "ArmRotate");
        ElevatorRotate = hardwareMap.get(DcMotor.class, "ElevatorRotate");
        Hook = hardwareMap.get(DcMotor.class, "Hook");
        Claw = hardwareMap.get(CRServo.class, "Claw");

        // Optionally, set the initial power of the CRServo to 0
        Claw.setPower(0);

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
        Elevator.setDirection(DcMotor.Direction.FORWARD);
        ArmRotate.setDirection(DcMotor.Direction.FORWARD);
        ElevatorRotate.setDirection(DcMotor.Direction.FORWARD);
        Hook.setDirection(DcMotor.Direction.FORWARD);

        //Set the elevator up to run to position using encoders
        Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElevatorRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ElevatorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Motor Throttle
        double motorSpeed = 0.5; // Current motor speed
        double minSpeed = -1.0; // Minimum motor speed
        double maxSpeed = 1.0; // Maximum motor speed
        double speedIncrement = 0.1; // Increment for speed adjustment

        int ElevatorTarget = 0;
        int ElevatorRotateTarget = 0;

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

            if ( gamepad1.dpad_up && ElevatorRotateTarget < 500 ) ElevatorRotateTarget += 5;
            else if ( gamepad1.dpad_down && ElevatorRotateTarget > -500 ) ElevatorRotateTarget -= 5;
            ElevatorRotate.setTargetPosition(ElevatorRotateTarget);
            ElevatorRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ElevatorRotate.setPower(1);
            telemetry.addData("ElevatorRotateTarget", ElevatorRotateTarget);

            if (gamepad1.dpad_right)
                ArmRotate.setPower(0.5);
            else if (gamepad1.dpad_left)
                ArmRotate.setPower(-0.5);
            else
                ArmRotate.setPower(0);

            if ( gamepad1.right_trigger != 0 && ElevatorTarget < 2000 ) ElevatorTarget += 5;
            else if ( gamepad1.left_trigger != 0 && ElevatorTarget > 1 ) ElevatorTarget -= 5;
            Elevator.setTargetPosition(ElevatorTarget);
            Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Elevator.setPower(1);
            telemetry.addData("ElevatorTarget", ElevatorTarget);

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

            // Check if the B button is pressed
            if (gamepad1.b) {
                // Set the CRServo to move (e.g., forward with 0.5 power)
                Claw.setPower(0);
            } else {
                // Stop the CRServo if the B button is not pressed
                Claw.setPower(0.5);
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
            telemetry.addData("Motor Speed",motorSpeed);
            telemetry.update();



        }
    }
}