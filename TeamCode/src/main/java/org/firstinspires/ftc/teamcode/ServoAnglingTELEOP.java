package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoAnglingTELEOP  ", group = "TeleOp")
public class ServoAnglingTELEOP extends OpMode {

    // Servos
    Servo leftServo;
    Servo rightServo;

    // Increment per loop (controls speed)
    final double INCREMENT = 0.02;

    // Servo max angle in degrees (your calibrated max)
    final double MAX_ANGLE = 200.0;

    // Current positions (0–1 range)
    double leftPos;
    double rightPos;

    @Override
    public void init() {
        // CHANGE THESE TO MATCH YOUR ROBOT CONFIG
        leftServo  = hardwareMap.get(Servo.class, "your name here");
        rightServo = hardwareMap.get(Servo.class, "your name here");

        // Reverse one servo because they face each other
        rightServo.setDirection(Servo.Direction.REVERSE);

        // RESET zero position at start
        leftPos = 0.0;    // fully out
        rightPos = 0.0;   // fully out
        leftServo.setPosition(leftPos);
        rightServo.setPosition(rightPos);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            // Move IN
            leftPos  = Math.min(leftPos + INCREMENT, 1.0);
            rightPos = Math.min(rightPos + INCREMENT, 1.0);

        } else if (gamepad1.dpad_down) {
            // Move OUT
            leftPos  = Math.max(leftPos - INCREMENT, 0.0);
            rightPos = Math.max(rightPos - INCREMENT, 0.0);
        }

        // Update servo positions
        leftServo.setPosition(leftPos);
        rightServo.setPosition(rightPos);

        // Telemetry: show 0–1 and degrees
        telemetry.addData("Left Position", "%.2f (%.1f deg)", leftPos, leftPos * MAX_ANGLE);
        telemetry.addData("Right Position", "%.2f (%.1f deg)", rightPos, rightPos * MAX_ANGLE);
        telemetry.update();
    }
}


