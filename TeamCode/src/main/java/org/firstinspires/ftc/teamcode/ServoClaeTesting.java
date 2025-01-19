package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Claw Works", group="Linear OpMode")
@Disabled
public class ServoClaeTesting extends LinearOpMode {
    private CRServo myServo;

    @Override
    public void runOpMode() {
        // Initialize the servo
        myServo = hardwareMap.get(CRServo.class, "claw");

        waitForStart();

        while (opModeIsActive()) {
            // Variable to store servo power
            double power = 0.0;

            if (gamepad1.a) {
                power = 1;  // Move forward
            } else if (gamepad1.b) {
                power = -1; // Move backward


                // Set the power to the servo
                myServo.setPower(power);

                // Telemetry for debugging purposes
                telemetry.addData("Button A", gamepad1.a);
                telemetry.addData("Button B", gamepad1.b);
                telemetry.addData("Servo Power", power);
                telemetry.update();
            }
        }
    }
}

