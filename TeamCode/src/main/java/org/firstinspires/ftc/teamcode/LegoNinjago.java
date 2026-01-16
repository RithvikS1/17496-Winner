package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.AprilTagItems.AprilTag;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "LegoNinjago")
public class LegoNinjago extends LinearOpMode {

    private DcMotor LB; // 0C
    private DcMotor LF; // 1C
    private DcMotor RB; // 2C
    private DcMotor RF; // 3C
    private DcMotorEx LSX; // 0E
    private DcMotorEx RSX; // 1E
    private DcMotorEx IntakeEx; // 2E

    // Drive power values
    double lbPower;
    double lfPower;
    double rbPower;
    double rfPower;
    boolean shooting=false;

    // Shooter velocity (tunable in FTC Dashboard)
    public static double SHOOTER_VELOCITY = 1425;
    public static double INTAKE_VELOCITY = 1800;


    // Odometry variables
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;

    static final double TICKS_PER_REV = 1120; // Example for NeverRest 40
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);

    // Track width (distance between left and right wheels, adjust to your robot)
    static final double TRACK_WIDTH = 12.0;

    int prevLB = 0, prevLF = 0, prevRB = 0, prevRF = 0;

    AprilTag cam;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Hardware mapping
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LSX = hardwareMap.get(DcMotorEx.class, "LS");
        RSX = hardwareMap.get(DcMotorEx.class, "RS");
        IntakeEx = hardwareMap.get(DcMotorEx.class, "Intake");

        // Motor directions
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);
        LSX.setDirection(DcMotor.Direction.REVERSE);
        RSX.setDirection(DcMotor.Direction.FORWARD);
        IntakeEx.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addLine("Initializing AprilTag Vision...");
        telemetry.update();

        // Create your AprilTag vision object
        cam = new AprilTag(hardwareMap, telemetry);

        telemetry.addLine("Vision Ready!");
        telemetry.update();


        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            shooterEx();
            intake();
            camera();

            // Mecanum drive calculations
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            lbPower = forward - strafe + turn;
            lfPower = forward + strafe + turn;
            rbPower = forward + strafe - turn;
            rfPower = forward - strafe - turn;

            // Normalize
            double max = Math.max(Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                    Math.max(Math.abs(lbPower), Math.abs(rbPower)));

            if (max > 1.0) {
                lbPower /= max;
                lfPower /= max;
                rbPower /= max;
                rfPower /= max;
            }

            // Set drive motor power
            LB.setPower(lbPower);
            LF.setPower(lfPower);
            RB.setPower(rbPower);
            RF.setPower(rfPower);

            // --- Odometry update ---
            int lb = LB.getCurrentPosition();
            int lf = LF.getCurrentPosition();
            int rb = RB.getCurrentPosition();
            int rf = RF.getCurrentPosition();

            int dLB = lb - prevLB;
            int dLF = lf - prevLF;
            int dRB = rb - prevRB;
            int dRF = rf - prevRF;

            prevLB = lb;
            prevLF = lf;
            prevRB = rb;
            prevRF = rf;

            double iLB = dLB / TICKS_PER_INCH;
            double iLF = dLF / TICKS_PER_INCH;
            double iRB = dRB / TICKS_PER_INCH;
            double iRF = dRF / TICKS_PER_INCH;

            // Local robot frame deltas
            double dX = (iLF - iRF - iLB + iRB) / 4.0; // strafe
            double dY = (iLF + iRF + iLB + iRB) / 4.0; // forward

            // Heading change from encoder difference
            double dTheta = ((iRF + iRB) - (iLF + iLB)) / (2.0 * TRACK_WIDTH);
            heading += dTheta;

            // Rotate into global coordinates
            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);

            x += dX * cosH - dY * sinH;
            y += dX * sinH + dY * cosH;
            heading *= 180/Math.PI;

            // Telemetry (Driver Station + Dashboard)
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Shooter Velocity:", SHOOTER_VELOCITY);
            telemetry.addData("Intake:", INTAKE_VELOCITY);
            telemetry.addData("LB:", lbPower);
            telemetry.addData("LF:", lfPower);
            telemetry.addData("RB:", rbPower);
            telemetry.addData("RF:", rfPower);
            telemetry.addData("Odometry:", "");
            telemetry.addData("X (in)", x);
            telemetry.addData("Y (in)", y);
            telemetry.addData("Heading (degrees)", heading);
            telemetry.update();

            heading *= Math.PI/180;
        }
        cam.cameraOff();
    }

    // Shooter control
    private void shooterEx() {
        if (gamepad1.dpad_up)
            SHOOTER_VELOCITY=1750;
        if (gamepad1.dpad_left)
            SHOOTER_VELOCITY=1425;
        if (gamepad1.dpad_down)
            SHOOTER_VELOCITY=400;

        if (gamepad1.a){
            LSX.setVelocity(SHOOTER_VELOCITY);
            RSX.setVelocity(SHOOTER_VELOCITY);
            shooting=true;
        }
        if (gamepad1.x){
            LSX.setVelocity(0);
            RSX.setVelocity(0);
            shooting = false;
        }
        if (gamepad1.b&&!shooting){
            LSX.setVelocity(-SHOOTER_VELOCITY);
            RSX.setVelocity(-SHOOTER_VELOCITY);
            shooting=true;
            sleep(10);
            LSX.setVelocity(0);
            RSX.setVelocity(0);
            shooting = false;
        }
    }

    // Intake control
    private void intake() {
        if (gamepad1.right_trigger>0)
            IntakeEx.setVelocity(INTAKE_VELOCITY);
        else if (gamepad1.left_trigger>0)
            IntakeEx.setVelocity(-INTAKE_VELOCITY);
        else
            IntakeEx.setVelocity(0);
    }

    private void camera(){
        cam.update();

        // Get the most recent detection
        AprilTagDetection tag = cam.getLatestTag();

        // ------------------------------
        // DRIVER STATION TELEMETRY
        // ------------------------------
        cam.addTelemetry();
        telemetry.update();

        // ------------------------------
        // FTC DASHBOARD TELEMETRY
        // ------------------------------
        cam.addDashboardTelemetry(tag);

        sleep(1);
    }
}
