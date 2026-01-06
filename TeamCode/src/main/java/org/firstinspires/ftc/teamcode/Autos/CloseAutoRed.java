package org.firstinspires.ftc.teamcode.Autos;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.AprilTagItems.AprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "CloseAutoRed")
public class CloseAutoRed extends LinearOpMode {

    // Drivetrain
    private DcMotorEx LF, LB, RF, RB;
    // Intake + Launcher
    private DcMotorEx IntakeEx, LSX, RSX;
    // Vision
    private AprilTag vision;

    public static long TURN_TIME = 1800;
    private static final double LAUNCHER_RPM = 1350;
    public static final double DEG_PER_SEC = 58;
    private static final double TURN_RPM = 860;
    private static double TURN_POWER = 0.17;
    private static final double TARGET_ANGLE_DEG = 45.0;
    private static final double TAG_WAIT_TIMEOUT = 2.0;

    private static final double SHOOTER_RPM = 1450;
    private static final double INTAKE_RPM1 = 1350;
    private static final double INTAKE_RPM2 = 800;

    private static final double TARGET_DIST_IN = 2450;

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
                    //sleep(120);

                    telemetry.addLine("Reached target distance.");
                    telemetry.update();
                    break;
                } else {
                    // CONSTANT SPEED BACKUP (your requirement)
                    drive(-700, -700, -700, -700);
                }
            } else {
                // STILL BACK UP AT SAME SPEED
                drive(-700, -700, -700, -700);
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

            //sleep(30);
        }

        drive(0,0,0,0);
        //sleep(100);

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
            //sleep(150);
        }

        if (!shooterReady) {
            telemetry.addLine("Shooter timed spin-up fallback");
            telemetry.update();
            //sleep(1500);
        } else {
            telemetry.addLine("Shooter ready");
            telemetry.update();
            //sleep(200);
        }

        telemetry.addLine("Feeding...");
        telemetry.update();

        IntakeEx.setVelocity(INTAKE_RPM1);
        sleep(1500);
        IntakeEx.setVelocity(0);
        sleep(200);
        IntakeEx.setVelocity(INTAKE_RPM2);
        sleep(1500);
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
            AprilTagDetection tag = vision.getTagById(24);

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

            //sleep(30);
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

        //sleep(300);

        // ==============================
        // PHASE 3: BLIND LEFT TURN
        // ==============================
        // Left turn: left motors backward, right motors forward
        drive(-TURN_RPM, -TURN_RPM, +TURN_RPM, +TURN_RPM);
        sleep(turnTimeMs);
        drive(0, 0, 0, 0);

        telemetry.addLine("Turn complete");
        telemetry.update();
        //sleep(250);

        // ---------------------------
        // STEP 0 — DRIVE FORWARD
        // ---------------------------
        drive(-700, -700, -700, -700);
        sleep(100);
        stopDrive();
        //sleep(50);
        // ---------------------------
        // STEP 0.1 — TURN TO BALLS
        // ---------------------------
        drive(600, 600, -600, -600);
        sleep(1600);
        stopDrive();

        // ---------------------------
        // STEP 0.2 — START INTAKE
        // ---------------------------
        IntakeEx.setPower(1.0);
        //sleep(50);

        // ---------------------------
        // STEP 0.3 — DRIVE INTO BALLS
        // ---------------------------
        drive(400, 400, 400, 400);
        sleep(4750);
        stopDrive();

        // ---------------------------
        // STEP 0.4 — INTAKE OFF
        // ---------------------------
        //IntakeEx.setPower(0);
        //sleep(25);

        // ---------------------------
        // STEP 0.5 — DRIVE BACKWARD
        // ---------------------------
        drive(-900, -900, -900, -900);
        sleep(100);
        IntakeEx.setVelocity(0);
        sleep(2500);
        stopDrive();
        //sleep(100);

        // ---------------------------
        // STEP 0.6 — TURN CCW UNTIL rawX
        // ---------------------------

        drive(-250, -250, 250, 250);

        long turnStartTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            vision.update();
            AprilTagDetection tag = vision.getTagById(24);

            if (tag != null && tag.rawPose != null) {
                double rawX = tag.rawPose.x;

                telemetry.addData("rawX", rawX);
                telemetry.update();

                // EXIT CONDITION — stop AND leave loop
                if (rawX >= -2.47) {
                    break;
                }
            }

            //sleep(15);
        }

        // Stop motors ONCE
        stopDrive();

        // Record duration AFTER loop exits
        long turnDuration = System.currentTimeMillis() - turnStartTime;


        // ---------------------------
        // STEP 0.7 — SLIGHT OUTTAKE
        // ---------------------------
        IntakeEx.setVelocity(-1800);
        LSX.setVelocity(-1350);
        RSX.setVelocity(-1350);
        sleep(500);
        LSX.setVelocity(0);
        RSX.setVelocity(0);
        IntakeEx.setPower(0);

        // ---------------------------
        // STEP 0.8 — SPIN UP LAUNCHERS
        // ---------------------------
        LSX.setVelocity(LAUNCHER_RPM);
        RSX.setVelocity(LAUNCHER_RPM);

        // ---------------------------
        // STEP 0.9 — FEED BALLS
        // ---------------------------
        IntakeEx.setPower(1.0);
        sleep(3000);
        IntakeEx.setPower(0);

        LSX.setVelocity(0);
        RSX.setVelocity(0);

        // ---------------------------
        // STEP 0.91 — TURN BACK CW + DRIVE FORWARD
        // ---------------------------
        drive(+TURN_RPM, +TURN_RPM, -TURN_RPM, -TURN_RPM);
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
        drive(0,0,0,0);
    }
}
