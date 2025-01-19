/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="NotMotorTest", group="Linear OpMode")
public class NotMotorTest extends LinearOpMode {

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

        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);
        Elevator.setDirection(DcMotor.Direction.REVERSE);
        ArmRotate.setDirection(DcMotor.Direction.FORWARD);
        ElevatorRotate.setDirection(DcMotor.Direction.FORWARD);

        //Set the elevator up to run to position using encoders
        Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        int ElevatorTarget = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*
            TODO: Create Driving Code, This was already here taking up space:
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double FrontLeftPower  = gamepad1.left_stick_y;
            double FrontRightPower = gamepad1.left_stick_x;
            double BackLeftPower   = gamepad1.right_stick_x;
            double BackRightPower  = gamepad1.right_stick_y;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(FrontLeftPower), Math.abs(FrontRightPower));
            max = Math.max(max, Math.abs(BackLeftPower));
            max = Math.max(max, Math.abs(BackRightPower));

            if (max > 1.0) {
                FrontLeftPower  /= max;
                FrontRightPower /= max;
                BackLeftPower   /= max;
                BackRightPower  /= max;
            }

             */
            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            FrontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            BackLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            FrontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            BackRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

            // Send calculated power to wheels
            FrontLeft.setPower(FrontLeftPower);
            FrontRight.setPower(FrontRightPower);
            BackLeft.setPower(BackLeftPower);
            BackRight.setPower(BackRightPower);
             */

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


            if ( gamepad1.right_trigger != 0 && ElevatorTarget < 4250 ) ElevatorTarget += 5;
            else if ( gamepad1.left_trigger != 0 && ElevatorTarget > 1 ) ElevatorTarget -= 5;
            Elevator.setTargetPosition(ElevatorTarget);
            Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Elevator.setPower(1);
            /*
                telemetry.addData("Target", ElevatorTarget);
                telemetry.update();
                */
            /*
            Elevator.setPower(gamepad1.right_trigger ? 1.0 : 0.0);
            ArmRotate.setPower(gamepad1.b ? 1.0 : 0.0);
            ElevatorRotate.setPower(gamepad1.y ? 1.0 : 0.0);
             */

            // Show the elapsed game time and wheel power.
            /*
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FrontLeft/Right", "%4.2f, %4.2f", FrontLeftPower, FrontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", BackLeftPower, BackRightPower);
            telemetry.update();
            */
        }
    }}
