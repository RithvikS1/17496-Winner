package org.firstinspires.ftc.teamcode.Autos.Close_PreReq;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AprilTagItems.AprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Config
@Autonomous(name = "BALL_COLLECTION")
public class BallCollection extends LinearOpMode {

    // Drivetrain
    private DcMotor LF, LB, RF, RB;

    // Intake + Launcher
    private DcMotorEx IntakeEx, LSX, RSX;


    // Vision
    private AprilTag vision;

    // Constants
    private static double TURN_POWER = 0.17;
    public static long TURN_TIME = 1200;
    private static final double LAUNCHER_RPM = 1400;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------------------
        // Hardware Init
        // ---------------------------
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");

        IntakeEx = hardwareMap.get(DcMotorEx.class, "Intake");
        LSX = hardwareMap.get(DcMotorEx.class, "LS");
        RSX = hardwareMap.get(DcMotorEx.class, "RS");

        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);

        IntakeEx.setDirection(DcMotorEx.Direction.REVERSE);
        LSX.setDirection(DcMotorEx.Direction.REVERSE);
        RSX.setDirection(DcMotorEx.Direction.FORWARD);

        vision = new AprilTag(hardwareMap, telemetry);

        waitForStart();
        if (isStopRequested()) return;

        // ---------------------------
        // STEP 0 — DRIVE FORWARD
        // ---------------------------
        drive(0.3, 0.3, 0.3, 0.3);
        sleep(400);
        stopDrive();

        // ---------------------------
        // STEP 0.1 — TURN TO BALLS
        // ---------------------------
        drive(0.4, 0.4, -0.4, -0.4);
        sleep(TURN_TIME);
        stopDrive();

        // ---------------------------
        // STEP 1 — START INTAKE
        // ---------------------------
        IntakeEx.setPower(1.0);

        // ---------------------------
        // STEP 2 — DRIVE INTO BALLS
        // ---------------------------
        drive(0.3, 0.3, 0.3, 0.3);
        sleep(2700);
        stopDrive();

        // ---------------------------
        // STEP 3 — INTAKE OFF
        // ---------------------------
        IntakeEx.setPower(0);
        sleep(25);

        // ---------------------------
        // STEP 4 — DRIVE BACKWARD
        // ---------------------------
        drive(-0.5, -0.5, -0.5, -0.5);
        sleep(1520);
        stopDrive();
        sleep(100);

        // ---------------------------
        // STEP 5 — TURN CCW UNTIL rawX <= -2.39
        // ---------------------------

        drive(-0.17, -0.17, 0.17, 0.17);

        long turnStartTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            vision.update();
            AprilTagDetection tag = vision.getTagById(24);

            if (tag != null && tag.rawPose != null) {
                double rawX = tag.rawPose.x;

                telemetry.addData("rawX", rawX);
                telemetry.update();

                // EXIT CONDITION — stop AND leave loop
                if (rawX >= -2.18) {
                    break;
                }
            }

            sleep(15);
        }

        // Stop motors ONCE
        stopDrive();

        // Record duration AFTER loop exits
        long turnDuration = System.currentTimeMillis() - turnStartTime;


        // ---------------------------
        // STEP 6 — SLIGHT OUTTAKE
        // ---------------------------
        IntakeEx.setPower(-0.2);
        sleep(300);
        IntakeEx.setPower(0);

        // ---------------------------
        // STEP 7 — SPIN UP LAUNCHERS
        // ---------------------------
        LSX.setVelocity(LAUNCHER_RPM);
        RSX.setVelocity(LAUNCHER_RPM);
        sleep(2000);

        // ---------------------------
        // STEP 8 — FEED BALLS
        // ---------------------------
        IntakeEx.setPower(1.0);
        sleep(1500);
        IntakeEx.setPower(0);

        LSX.setVelocity(0);
        RSX.setVelocity(0);

        // ---------------------------
        // STEP 9 — TURN BACK CW (SAME AMOUNT) + a bit more to make the 90
        // ---------------------------
        drive(TURN_POWER, TURN_POWER, -TURN_POWER, -TURN_POWER);
        sleep(turnDuration);
        TURN_POWER = 0.3;
        drive(TURN_POWER, TURN_POWER, -TURN_POWER, -TURN_POWER);
        sleep(70);
        stopDrive();
    }

    // ================= Helper Methods =================

    private void drive(double lf, double lb, double rf, double rb) {
        LF.setPower(lf);
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);
    }

    private void stopDrive() {
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }
}
