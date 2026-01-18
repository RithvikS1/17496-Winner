package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.acmerobotics.dashboard.config.Config;

/**
 * BIG R TELEOP — MOVEMENT ONLY
 *
 * Drivetrain only. No odometry, no shooters, no intake, no vision.
 * Odometry pods / Road Runner localization handled elsewhere.
 *
 * Controls are intentionally minimal and competition-safe.
 */
@Config
@TeleOp(name = "BigRTeleOp", group = "TeleOp")
public class BigRTeleOp extends LinearOpMode {

    // -------------------------
    // HARDWARE
    // -------------------------
    private DcMotor LB, LF, RB, RF;

    // -------------------------
    // DASHBOARD TUNABLES
    // -------------------------
    public static double PRECISION_MULT = 0.35;
    public static double NORMAL_MULT    = 0.75;
    public static double TURBO_MULT     = 1.00;

    public static double DZ_TRANSLATE = 0.05;
    public static double DZ_TURN      = 0.05;

    // -------------------------
    // DRIVE MODE
    // -------------------------
    private enum DriveMode { PRECISION, NORMAL, TURBO }
    private DriveMode driveMode = DriveMode.NORMAL;

    // Drive powers
    double lbPower, lfPower, rbPower, rfPower;

    @Override
    public void runOpMode() {

        // -------------------------
        // HARDWARE MAP
        // -------------------------
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");

        // -------------------------
        // MOTOR DIRECTIONS (your exact setup)
        // -------------------------
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Big R TeleOp Initialized");
        telemetry.addLine("Movement ONLY — Odometry handled elsewhere");
        telemetry.addLine("D-Pad selects drive mode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // -------------------------
            // DRIVE MODE SELECT (D-PAD)
            // -------------------------
            if (gamepad1.dpad_left)  setDriveMode(DriveMode.PRECISION);
            if (gamepad1.dpad_up)    setDriveMode(DriveMode.NORMAL);
            if (gamepad1.dpad_right) setDriveMode(DriveMode.TURBO);

            // -------------------------
            // MECANUM DRIVE
            // -------------------------
            double forward = deadzone(-gamepad1.left_stick_y, DZ_TRANSLATE);
            double strafe  = deadzone( gamepad1.left_stick_x, DZ_TRANSLATE);
            double turn    = deadzone( gamepad1.right_stick_x, DZ_TURN);

            double speedMult = getDriveMultiplier(driveMode);

            lbPower = forward - strafe + turn;
            lfPower = forward + strafe + turn;
            rbPower = forward + strafe - turn;
            rfPower = forward - strafe - turn;

            // Normalize
            double max = Math.max(
                    Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower))
            );

            if (max > 1.0) {
                lbPower /= max;
                lfPower /= max;
                rbPower /= max;
                rfPower /= max;
            }

            LB.setPower(lbPower * speedMult);
            LF.setPower(lfPower * speedMult);
            RB.setPower(rbPower * speedMult);
            RF.setPower(rfPower * speedMult);

            telemetry.addData("Drive Mode", driveMode);
            telemetry.update();
        }
    }

    // -------------------------
    // HELPERS
    // -------------------------
    private void setDriveMode(DriveMode mode) {
        if (driveMode != mode) {
            driveMode = mode;

            // Rumble feedback for driver confirmation
            if (mode == DriveMode.PRECISION) gamepad1.rumbleBlips(1);
            if (mode == DriveMode.NORMAL)    gamepad1.rumbleBlips(2);
            if (mode == DriveMode.TURBO)     gamepad1.rumbleBlips(3);
        }
    }

    private double getDriveMultiplier(DriveMode mode) {
        switch (mode) {
            case PRECISION: return PRECISION_MULT;
            case NORMAL:    return NORMAL_MULT;
            case TURBO:     return TURBO_MULT;
            default:        return NORMAL_MULT;
        }
    }

    private double deadzone(double v, double dz) {
        return Math.abs(v) < dz ? 0.0 : v;
    }
}

/*
================================================================================
BIG R TELEOP — CONTROLS LIST (PS5 DUALSENSE)
================================================================================

DRIVING:
- Left Stick Y        → Forward / Backward
- Left Stick X        → Strafe Left / Right
- Right Stick X       → Turn

DRIVE MODES:
- D-Pad Left          → PRECISION mode (slow, accurate)
- D-Pad Up            → NORMAL mode
- D-Pad Right         → TURBO mode (full speed)
  (Rumble confirms mode change)

NOT INCLUDED:
- ❌ Odometry
- ❌ Shooter
- ❌ Intake
- ❌ Vision
- ❌ Touchpad logic

Odometry pods + localization are handled in a separate subsystem.
================================================================================
*/
