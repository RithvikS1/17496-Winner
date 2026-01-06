package org.firstinspires.ftc.teamcode.Autos.Close_PreReq;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

@Config
@TeleOp(name = "Mecanum Calibration", group = "Calibration")
public class MecanumCalibration extends LinearOpMode {

    private DcMotor LB, LF, RB, RF;

    // Tunable test parameters
    public static double TEST_POWER = 0.5;
    public static double TEST_DURATION_MS = 2000;

    // Calculated offsets (will be computed during calibration)
    public static double LB_OFFSET = 1.0;
    public static double LF_OFFSET = 1.0;
    public static double RB_OFFSET = 1.0;
    public static double RF_OFFSET = 1.0;

    private enum CalibrationState {
        INSTRUCTIONS,
        INDIVIDUAL_MOTOR_TEST,
        STRAFE_RIGHT_TEST,
        STRAFE_LEFT_TEST,
        RESULTS,
        LIVE_TUNING
    }

    private CalibrationState state = CalibrationState.INSTRUCTIONS;
    private int testStep = 0;
    private boolean motorHasRun = false;
    private boolean aPressed = false;
    private boolean bPressed = false;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Hardware mapping
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");

        // Motor directions (match your main code)
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        resetEncoders();

        telemetry.addData("Status", "Ready for Calibration");
        telemetry.addData("Instructions", "Press A to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Button state tracking (prevents double-press)
            boolean aCurrentlyPressed = gamepad1.a;
            boolean bCurrentlyPressed = gamepad1.b;

            switch (state) {
                case INSTRUCTIONS:
                    showInstructions(aCurrentlyPressed);
                    break;
                case INDIVIDUAL_MOTOR_TEST:
                    runIndividualMotorTest(aCurrentlyPressed, bCurrentlyPressed);
                    break;
                case STRAFE_RIGHT_TEST:
                    runStrafeTest(true, aCurrentlyPressed, bCurrentlyPressed);
                    break;
                case STRAFE_LEFT_TEST:
                    runStrafeTest(false, aCurrentlyPressed, bCurrentlyPressed);
                    break;
                case RESULTS:
                    showResults(aCurrentlyPressed);
                    break;
                case LIVE_TUNING:
                    runLiveTuning();
                    break;
            }

            // Update button states
            aPressed = aCurrentlyPressed;
            bPressed = bCurrentlyPressed;

            telemetry.update();
        }
    }

    private void showInstructions(boolean aCurrentlyPressed) {
        telemetry.addLine("=== MECANUM CALIBRATION ===");
        telemetry.addLine();
        telemetry.addLine("This will test each motor and");
        telemetry.addLine("calculate strafe offsets.");
        telemetry.addLine();
        telemetry.addLine("SETUP:");
        telemetry.addLine("1. Place robot on field");
        telemetry.addLine("2. Mark starting position");
        telemetry.addLine("3. Ensure wheels can move freely");
        telemetry.addLine();
        telemetry.addLine(">>> Press A to start motor tests <<<");

        if (aCurrentlyPressed && !aPressed) {
            state = CalibrationState.INDIVIDUAL_MOTOR_TEST;
            testStep = 0;
            motorHasRun = false;
        }
    }

    private void runIndividualMotorTest(boolean aCurrentlyPressed, boolean bCurrentlyPressed) {
        String[] motorNames = {"LB", "LF", "RB", "RF"};
        DcMotor[] motors = {LB, LF, RB, RF};

        if (testStep >= motors.length) {
            state = CalibrationState.STRAFE_RIGHT_TEST;
            testStep = 0;
            motorHasRun = false;
            resetEncoders();
            sleep(500);
            return;
        }

        telemetry.addLine("=== INDIVIDUAL MOTOR TEST ===");
        telemetry.addLine();
        telemetry.addData("Testing Motor", motorNames[testStep] + " (" + (testStep + 1) + " of 4)");
        telemetry.addLine();
        telemetry.addLine("Watch the motor spin and note:");
        telemetry.addLine("- Does it spin smoothly?");
        telemetry.addLine("- Any unusual sounds?");
        telemetry.addLine("- Encoder count should increase");
        telemetry.addLine();

        if (!motorHasRun) {
            telemetry.addData("Current Encoder", motors[testStep].getCurrentPosition());
            telemetry.addLine();
            telemetry.addLine(">>> Press A to run motor <<<");

            if (aCurrentlyPressed && !aPressed) {
                // Run the motor
                int startPos = motors[testStep].getCurrentPosition();
                motors[testStep].setPower(TEST_POWER);
                sleep((long)TEST_DURATION_MS);
                motors[testStep].setPower(0);
                int endPos = motors[testStep].getCurrentPosition();

                motorHasRun = true;
                telemetry.addData("Distance Traveled", Math.abs(endPos - startPos) + " ticks");
                telemetry.update();
            }
        } else {
            int currentPos = motors[testStep].getCurrentPosition();
            telemetry.addData("Final Encoder", currentPos);
            telemetry.addData("Distance Traveled", Math.abs(currentPos) + " ticks");
            telemetry.addLine();
            telemetry.addLine(">>> Press B to test next motor <<<");

            if (bCurrentlyPressed && !bPressed) {
                motorHasRun = false;
                testStep++;
            }
        }
    }

    private void runStrafeTest(boolean strafeRight, boolean aCurrentlyPressed, boolean bCurrentlyPressed) {
        String direction = strafeRight ? "RIGHT" : "LEFT";

        telemetry.addLine("=== STRAFE " + direction + " TEST ===");
        telemetry.addLine();
        telemetry.addLine("Robot will strafe " + direction.toLowerCase());
        telemetry.addLine("Watch if it rotates or moves straight");
        telemetry.addLine();
        telemetry.addData("Test Duration", TEST_DURATION_MS + "ms");
        telemetry.addData("Test Power", TEST_POWER);
        telemetry.addLine();

        if (!motorHasRun) {
            telemetry.addLine(">>> Press A to run test <<<");

            if (aCurrentlyPressed && !aPressed) {
                // Reset encoders
                resetEncoders();
                sleep(100);

                int startLB = LB.getCurrentPosition();
                int startLF = LF.getCurrentPosition();
                int startRB = RB.getCurrentPosition();
                int startRF = RF.getCurrentPosition();

                // Pure strafe motion
                double strafe = strafeRight ? 1.0 : -1.0;
                LB.setPower(-strafe * TEST_POWER);
                LF.setPower(strafe * TEST_POWER);
                RB.setPower(strafe * TEST_POWER);
                RF.setPower(-strafe * TEST_POWER);

                sleep((long)TEST_DURATION_MS);

                LB.setPower(0);
                LF.setPower(0);
                RB.setPower(0);
                RF.setPower(0);

                int endLB = LB.getCurrentPosition();
                int endLF = LF.getCurrentPosition();
                int endRB = RB.getCurrentPosition();
                int endRF = RF.getCurrentPosition();

                int deltaLB = Math.abs(endLB - startLB);
                int deltaLF = Math.abs(endLF - startLF);
                int deltaRB = Math.abs(endRB - startRB);
                int deltaRF = Math.abs(endRF - startRF);

                // Calculate offsets - normalize to the wheel that moved the most
                double maxDelta = Math.max(Math.max(deltaLB, deltaLF), Math.max(deltaRB, deltaRF));

                LB_OFFSET = maxDelta / deltaLB;
                LF_OFFSET = maxDelta / deltaLF;
                RB_OFFSET = maxDelta / deltaRB;
                RF_OFFSET = maxDelta / deltaRF;

                motorHasRun = true;
            }
        } else {
            telemetry.addLine("RESULTS:");
            telemetry.addLine();
            telemetry.addLine("Calculated Offsets:");
            telemetry.addData("LB_OFFSET", String.format("%.3f", LB_OFFSET));
            telemetry.addData("LF_OFFSET", String.format("%.3f", LF_OFFSET));
            telemetry.addData("RB_OFFSET", String.format("%.3f", RB_OFFSET));
            telemetry.addData("RF_OFFSET", String.format("%.3f", RF_OFFSET));
            telemetry.addLine();
            telemetry.addLine(">>> Press B to continue <<<");

            if (bCurrentlyPressed && !bPressed) {
                motorHasRun = false;
                if (strafeRight) {
                    state = CalibrationState.STRAFE_LEFT_TEST;
                } else {
                    state = CalibrationState.RESULTS;
                }
            }
        }
    }

    private void showResults(boolean aCurrentlyPressed) {
        telemetry.addLine("=== CALIBRATION COMPLETE ===");
        telemetry.addLine();
        telemetry.addLine("FINAL OFFSETS:");
        telemetry.addData("LB_STRAFE_OFFSET", String.format("%.3f", LB_OFFSET));
        telemetry.addData("LF_STRAFE_OFFSET", String.format("%.3f", LF_OFFSET));
        telemetry.addData("RB_STRAFE_OFFSET", String.format("%.3f", RB_OFFSET));
        telemetry.addData("RF_STRAFE_OFFSET", String.format("%.3f", RF_OFFSET));
        telemetry.addLine();
        telemetry.addLine("Copy these to your main code:");
        telemetry.addLine("public static double LB_STRAFE_OFFSET=" + String.format("%.3f", LB_OFFSET) + ";");
        telemetry.addLine("public static double LF_STRAFE_OFFSET=" + String.format("%.3f", LF_OFFSET) + ";");
        telemetry.addLine("public static double RB_STRAFE_OFFSET=" + String.format("%.3f", RB_OFFSET) + ";");
        telemetry.addLine("public static double RF_STRAFE_OFFSET=" + String.format("%.3f", RF_OFFSET) + ";");
        telemetry.addLine();
        telemetry.addLine(">>> Press A for live tuning test <<<");

        if (aCurrentlyPressed && !aPressed) {
            state = CalibrationState.LIVE_TUNING;
        }
    }

    private void runLiveTuning() {
        telemetry.addLine("=== LIVE TUNING MODE ===");
        telemetry.addLine();
        telemetry.addLine("Use FTC Dashboard to adjust offsets");
        telemetry.addLine("Drive with left stick to test");
        telemetry.addLine();

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double lbPower = forward - LB_OFFSET * strafe + turn;
        double lfPower = forward + LF_OFFSET * strafe + turn;
        double rbPower = forward + RB_OFFSET * strafe - turn;
        double rfPower = forward - RF_OFFSET * strafe - turn;

        // Normalize
        double max = Math.max(Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                Math.max(Math.abs(lbPower), Math.abs(rbPower)));
        if (max > 1.0) {
            lbPower /= max;
            lfPower /= max;
            rbPower /= max;
            rfPower /= max;
        }

        LB.setPower(lbPower);
        LF.setPower(lfPower);
        RB.setPower(rbPower);
        RF.setPower(rfPower);

        telemetry.addData("LB Power", String.format("%.2f", lbPower));
        telemetry.addData("LF Power", String.format("%.2f", lfPower));
        telemetry.addData("RB Power", String.format("%.2f", rbPower));
        telemetry.addData("RF Power", String.format("%.2f", rfPower));
        telemetry.addLine();
        telemetry.addData("LB_OFFSET", String.format("%.3f", LB_OFFSET));
        telemetry.addData("LF_OFFSET", String.format("%.3f", LF_OFFSET));
        telemetry.addData("RB_OFFSET", String.format("%.3f", RB_OFFSET));
        telemetry.addData("RF_OFFSET", String.format("%.3f", RF_OFFSET));
    }

    private void resetEncoders() {
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}