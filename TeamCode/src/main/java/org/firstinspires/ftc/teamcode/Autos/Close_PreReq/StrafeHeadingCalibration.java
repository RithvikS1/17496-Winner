package org.firstinspires.ftc.teamcode.Autos.Close_PreReq;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "Strafe Heading Calibration", group = "Calibration")
public class StrafeHeadingCalibration extends LinearOpMode {

    // TUNE THIS
    public static double STRAFE_kP = 2.0;
    public static double STRAFE_POWER = 0.4;

    // Your known offsets
    public static double LB_OFFSET = 1.050;
    public static double LF_OFFSET = 1.027;
    public static double RB_OFFSET = 1.037;
    public static double RF_OFFSET = 1.000;

    DcMotor LB, LF, RB, RF;
    IMU imu;

    double targetHeading = 0;
    boolean wasStrafing = false;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");

        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        telemetry.addLine("Strafe Heading Calibration");
        telemetry.addLine("Use DPAD LEFT / RIGHT");
        telemetry.addLine("Tune STRAFE_kP in Dashboard");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean strafing = gamepad1.dpad_left || gamepad1.dpad_right;

            if (strafing && !wasStrafing) {
                targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }
            wasStrafing = strafing;

            double strafe = 0;
            if (gamepad1.dpad_left)  strafe = -1;
            if (gamepad1.dpad_right) strafe = 1;

            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double error = targetHeading - currentHeading;
            error = Math.atan2(Math.sin(error), Math.cos(error));

            double turnCorrection = STRAFE_kP * error;

            double lb = -LB_OFFSET * strafe * STRAFE_POWER + turnCorrection;
            double lf =  LF_OFFSET * strafe * STRAFE_POWER + turnCorrection;
            double rb =  RB_OFFSET * strafe * STRAFE_POWER - turnCorrection;
            double rf = -RF_OFFSET * strafe * STRAFE_POWER - turnCorrection;

            double max = Math.max(
                    Math.max(Math.abs(lb), Math.abs(lf)),
                    Math.max(Math.abs(rb), Math.abs(rf))
            );
            if (max > 1.0) {
                lb /= max;
                lf /= max;
                rb /= max;
                rf /= max;
            }

            LB.setPower(lb);
            LF.setPower(lf);
            RB.setPower(rb);
            RF.setPower(rf);

            telemetry.addData("Heading Error (deg)", Math.toDegrees(error));
            telemetry.addData("STRAFE_kP", STRAFE_kP);
            telemetry.update();
        }
    }
}
