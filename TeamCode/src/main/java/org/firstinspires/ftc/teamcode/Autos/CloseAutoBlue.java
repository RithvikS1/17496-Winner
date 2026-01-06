package org.firstinspires.ftc.teamcode.Autos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AprilTagItems.AprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "CloseAutoBlue")
public class CloseAutoBlue extends LinearOpMode {

    // Drivetrain
    private DcMotorEx LF, LB, RF, RB;
    // Intake + Launcher
    private DcMotorEx IntakeEx, LSX, RSX;
    // Vision
    private AprilTag vision;

    public static long TURN_TIME = 1960;
    private static final double LAUNCHER_RPM = 1370;
    public static final double DEG_PER_SEC = 58;
    private static final double TURN_RPM = 860;
    private static double TURN_POWER = 0.17;
    private static final double TARGET_ANGLE_DEG = 45.0;
    private static final double TAG_WAIT_TIMEOUT = 2.0;

    private static final double SHOOTER_RPM = 1450;
    private static final double INTAKE_RPM1 = 1350;
    private static final double INTAKE_RPM2 = 800;

    private static final double TARGET_DIST_IN = 2280;

    @Override
    public void runOpMode() throws InterruptedException {

        // ==============================
        // HARDWARE MAP
        // ==============================
        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RB = hardwareMap.get(DcMotorEx.class, "RB");
        IntakeEx = hardwareMap.get(DcMotorEx.class, "Intake");
        LSX = hardwareMap.get(DcMotorEx.class, "LS");
        RSX = hardwareMap.get(DcMotorEx.class, "RS");

        LF.setDirection(DcMotorEx.Direction.REVERSE);
        LB.setDirection(DcMotorEx.Direction.FORWARD);
        RF.setDirection(DcMotorEx.Direction.FORWARD);
        RB.setDirection(DcMotorEx.Direction.FORWARD);
        IntakeEx.setDirection(DcMotorEx.Direction.REVERSE);
        LSX.setDirection(DcMotorEx.Direction.REVERSE);
        RSX.setDirection(DcMotorEx.Direction.FORWARD);

        telemetry.addLine("Initializing AprilTag camera...");
        telemetry.update();

        vision = new AprilTag(hardwareMap, telemetry);

        telemetry.addLine("Ready. Press PLAY.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double startA = getRuntime();
        double timeoutA = 1;

        while (opModeIsActive()) {
            vision.update();
            AprilTagDetection tag = vision.getLatestTag();
            vision.addTelemetry();
            vision.addDashboardTelemetry(tag);
            telemetry.update();

            if (tag != null && vision.isStable()) {
                telemetry.addLine("TAG STABLE — applying brake pulse");
                telemetry.update();

                drive(+200, +200, +200, +200);
                sleep(120);
                drive(0,0,0,0);
                sleep(150);
                break;
            }

            if (getRuntime() - startA > timeoutA) {
                telemetry.addLine("Timeout waiting for stable tag — proceeding");
                telemetry.update();
                break;
            }

            sleep(30);
        }

        // ===============================================
        // PHASE B – BACK UP CONSTANT SPEED until distance
        // ===============================================
        telemetry.addLine("Phase B: backing up until distance > " + TARGET_DIST_IN);
        telemetry.update();

        double startB = getRuntime();
        double timeoutB = 6.0;

        while (opModeIsActive()) {

            vision.update();
            AprilTagDetection tag = vision.getLatestTag();

            double smoothedDist = vision.getSmoothedDistanceInches();

            if (smoothedDist > 0) {
                telemetry.addData("SmoothedDist(in)", "%.2f", smoothedDist);

                if (smoothedDist > TARGET_DIST_IN) {
                    // INSTANT BRAKE
                    drive(0,0,0,0);
                    sleep(120);

                    telemetry.addLine("Reached target distance.");
                    telemetry.update();
                    break;
                } else {
                    // CONSTANT SPEED BACKUP (your requirement)
                    drive(-1100, -1100, -1100, -1100);
                }
            } else {
                // STILL BACK UP AT SAME SPEED
                drive(-1100, -1100, -1100, -1100);
            }

            vision.addTelemetry();
            vision.addDashboardTelemetry(tag);
            telemetry.update();

            if (getRuntime() - startB > timeoutB) {
                telemetry.addLine("Backup timeout — moving to shoot");
                telemetry.update();
                drive(0,0,0,0);
                break;
            }

            sleep(30);
        }

        drive(0,0,0,0);
        sleep(100);

        // =====================
        // SHOOTER ROUTINE
        // =====================
        telemetry.addLine("Spinning shooter...");
        telemetry.update();

        LSX.setVelocity(SHOOTER_RPM);
        RSX.setVelocity(SHOOTER_RPM);

        double shooterStart = getRuntime();
        double shooterTimeout = 5.0;
        boolean shooterReady = false;

        while (opModeIsActive() && (getRuntime() - shooterStart < shooterTimeout)) {
            try {
                double vLS = Math.abs(LSX.getVelocity());
                double vRS = Math.abs(RSX.getVelocity());
                telemetry.addData("LS vel", "%.0f", vLS);
                telemetry.addData("RS vel", "%.0f", vRS);

                if (vLS >= 0.90 * SHOOTER_RPM && vRS >= 0.90 * SHOOTER_RPM) {
                    shooterReady = true;
                    break;
                }
            } catch (Exception e) {}

            telemetry.update();
            sleep(150);
        }

        if (!shooterReady) {
            telemetry.addLine("Shooter timed spin-up fallback");
            telemetry.update();
            sleep(1500);
        } else {
            telemetry.addLine("Shooter ready");
            telemetry.update();
            sleep(200);
        }

        telemetry.addLine("Feeding...");
        telemetry.update();

        IntakeEx.setVelocity(INTAKE_RPM1);
        sleep(1500);
        IntakeEx.setVelocity(0);
        sleep(200);
        IntakeEx.setVelocity(INTAKE_RPM2);
        sleep(1800);
        IntakeEx.setVelocity(0);

        LSX.setVelocity(0);
        RSX.setVelocity(0);

        // ==============================
        // PHASE 1: WAIT FOR STABLE TAG
        // ==============================
        double savedThetaAbsDeg = 0.0;
        boolean gotAngle = false;

        double startTime = getRuntime();

        while (opModeIsActive()) {
            vision.update();
            AprilTagDetection tag = vision.getTagById(20);

            if (tag != null && vision.isStable()) {
                double theta = vision.getAngleDegrees(tag);
                savedThetaAbsDeg = Math.abs(theta);
                gotAngle = true;

                telemetry.addData("Saved theta (abs deg)", "%.2f", savedThetaAbsDeg);
                telemetry.update();
                break;
            }

            if (getRuntime() - startTime > TAG_WAIT_TIMEOUT) {
                telemetry.addLine("Tag timeout — using 0°");
                telemetry.update();
                break;
            }

            sleep(10);
        }

        // ==============================
        // PHASE 2: COMPUTE TURN
        // ==============================
        double turnDegrees = TARGET_ANGLE_DEG - savedThetaAbsDeg;
        turnDegrees = Math.max(0.0, turnDegrees); // force positive

        double turnTimeSec = turnDegrees / DEG_PER_SEC;
        long turnTimeMs = (long)(turnTimeSec * 1000);

        telemetry.addData("Turn degrees", "%.2f", turnDegrees);
        telemetry.addData("Turn time (ms)", turnTimeMs);
        telemetry.update();

        sleep(50);

        // ==============================
        // PHASE 3: BLIND LEFT TURN
        // ==============================
        // Right
        drive(TURN_RPM, TURN_RPM, -TURN_RPM, -TURN_RPM);
        sleep(turnTimeMs);
        drive(0, 0, 0, 0);

        telemetry.addLine("Turn complete");
        telemetry.update();
        sleep(100);

        // ---------------------------
        // STEP 0 — DRIVE FORWARD
        // ---------------------------
        drive(700, 700, 700, 700);
        sleep(100);
        stopDrive();
        sleep(50);
        // ---------------------------
        // STEP 0.1 — TURN TO BALLS
        // ---------------------------
        drive(-600, -600, 600, 600);
        sleep(TURN_TIME);
        stopDrive();

        // ---------------------------
        // STEP 0.2 — START INTAKE
        // ---------------------------
        IntakeEx.setPower(1.0);
        sleep(30);

        // ---------------------------
        // STEP 0.3 — DRIVE INTO BALLS
        // ---------------------------
        drive(450, 450, 450, 450);
        sleep(3600);
        stopDrive();

        // ---------------------------
        // STEP 0.4 — INTAKE OFF
        // ---------------------------
        IntakeEx.setPower(0);
        sleep(10);

        // ---------------------------
        // STEP 0.5 — DRIVE BACKWARD
        // ---------------------------
        drive(-900, -900, -900, -900);
        sleep(1650);
        stopDrive();
        sleep(10);

        // ---------------------------
        // STEP 0.6 — TURN CW UNTIL rawX
        // ---------------------------

        drive(160, 160, -160, -160);

        long turnStartTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            vision.update();
            AprilTagDetection tag = vision.getTagById(20);

            if (tag != null && tag.rawPose != null) {
                double rawX = tag.rawPose.x;

                telemetry.addData("rawX", rawX);
                telemetry.update();

                // EXIT CONDITION — stop AND leave loop
                if (rawX <= 5.5) {
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
        // STEP 0.7 — SLIGHT OUTTAKE
        // ---------------------------
        IntakeEx.setPower(-0.2);
        sleep(300);
        IntakeEx.setPower(0);

        // ---------------------------
        // STEP 0.8 — SPIN UP LAUNCHERS
        // ---------------------------
        LSX.setVelocity(LAUNCHER_RPM);
        RSX.setVelocity(LAUNCHER_RPM);
        sleep(2300);

        // ---------------------------
        // STEP 0.9 — FEED BALLS
        // ---------------------------
        IntakeEx.setPower(1.0);
        sleep(2300);
        IntakeEx.setPower(0);

        LSX.setVelocity(0);
        RSX.setVelocity(0);
/**
 // ---------------------------
 // STEP 0.91 — TURN BACK CW (SAME AMOUNT) + a bit more to make the 90
 // ---------------------------
 drive(TURN_POWER, TURN_POWER, -TURN_POWER, -TURN_POWER);
 sleep(turnDuration);
 TURN_POWER = 0.3;
 drive(TURN_RPM, TURN_RPM, -TURN_RPM, -TURN_RPM);
 sleep(70);
 stopDrive();
 */

        // ---------------------------
        // STEP 0.91 — TURN BACK CCW + DRIVE FORWARD
        // ---------------------------
        drive(-800, -800, 800, 800);
        sleep(1500);
        drive(1000,1000,1000,1000);
        sleep(1500);
    }

    // ==============================
    // DRIVE HELPERS
    // ==============================
    private void drive(double lf, double lb, double rf, double rb) {
        LF.setVelocity(lf);
        LB.setVelocity(lb);
        RF.setVelocity(rf);
        RB.setVelocity(rb);
    }
    private void stopDrive() {
        LF.setVelocity(0);
        LB.setVelocity(0);
        RF.setVelocity(0);
        RB.setVelocity(0);
    }
}
