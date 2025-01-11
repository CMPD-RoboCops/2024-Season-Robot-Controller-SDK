package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="claw works", group="Linear OpMode")

public class ServoClaeTesting extends LinearOpMode {
    private Servo myServo;

    @Override
    public void runOpMode() {
        // Initialize the servo
        myServo = hardwareMap.get(Servo.class, "myServo");

        while (opModeIsActive()) {
            // Check if the A button is pressed
            if (gamepad1.a) {
                // Rotate the servo clockwise (move to position 1.0)
                myServo.setPosition(1.0);
            }
            // Check if the B button is pressed
            else if (gamepad1.b) {
                // Rotate the servo anticlockwise (move to position 0.0)
                myServo.setPosition(0.0);
            }

            // Sleep to prevent excessive looping
            sleep(20);
        }
    }

}