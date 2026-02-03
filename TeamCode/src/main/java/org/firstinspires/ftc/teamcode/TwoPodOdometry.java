package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * ===============================================================
 * TwoPodOdometry
 * ---------------------------------------------------------------
 * Purpose:
 *   Track the robot's (x, y, heading) position on the FIELD
 *   using:
 *     - ONE forward-facing odometry wheel
 *     - ONE sideways (strafe) odometry wheel
 *     - The Control Hub's built-in IMU (gyro)
 *
 * Coordinate system:
 *   - +X = field right
 *   - +Y = field forward
 *   - Heading = radians, CCW positive
 *
 * IMPORTANT:
 *   This is a 2-pod odometry system.
 *   Rotation is NOT solved by encoders — it is provided by the IMU.
 * ===============================================================
 */
public class TwoPodOdometry {

    // ===========================================================
    // 1) ENCODER + WHEEL CONSTANTS
    // ===========================================================

    /**
     * TICKS_PER_REV
     * -----------------------------------------------------------
     * Number of encoder ticks per full rotation of the odometry wheel.
     *
     * For REV Through-Bore encoders:
     *   - This is ALWAYS 8192 ticks per revolution.
     */
    public static final double TICKS_PER_REV = 8192;

    /**
     * WHEEL_DIAMETER_IN
     * -----------------------------------------------------------
     * Diameter of the odometry wheel in INCHES.
     *
     * HOW TO MEASURE:
     *   - Measure the wheel directly with calipers or ruler.
     *   - If unsure, start with manufacturer spec and tune later.
     *
     * WHY THIS MATTERS:
     *   Converts encoder ticks → inches of travel.
     */
    public static final double WHEEL_DIAMETER_IN = 0.0; // MEASURE THIS


    // ===========================================================
    // 2) ENCODER DIRECTION FIXES
    // ===========================================================

    /**
     * FORWARD_DIR
     * -----------------------------------------------------------
     * Direction multiplier for the FORWARD odometry pod.
     *
     * HOW TO TEST:
     *   - Push robot forward by hand.
     *   - If encoder value INCREASES → leave as +1
     *   - If encoder value DECREASES → set to -1
     */
    public static final int FORWARD_DIR = 1;

    /**
     * STRAFE_DIR
     * -----------------------------------------------------------
     * Direction multiplier for the STRAFE (sideways) odometry pod.
     *
     * HOW TO TEST:
     *   - Push robot left by hand.
     *   - If encoder value INCREASES → leave as +1
     *   - If encoder value DECREASES → set to -1
     */
    public static final int STRAFE_DIR = 1;


    // ===========================================================
    // 3) POD OFFSET MEASUREMENTS (CRITICAL)
    // ===========================================================

    /**
     * FORWARD_OFFSET_IN
     * -----------------------------------------------------------
     * Sideways distance from robot center to the FORWARD pod.
     *
     * WHAT THIS IS:
     *   - Measure LEFT/RIGHT distance from robot center
     *     to the forward pod wheel contact point.
     *
     * WHY IT EXISTS:
     *   - When robot ROTATES, the forward pod moves in an arc.
     *   - This offset lets us subtract rotation-induced motion.
     *
     * SIGN:
     *   - Left of center → positive or negative depending on convention
     *   - Start with magnitude only, tune sign if needed
     */
    public static final double FORWARD_OFFSET_IN = 0.0; // MEASURE THIS


    /**
     * STRAFE_OFFSET_IN
     * -----------------------------------------------------------
     * Forward/back distance from robot center to the STRAFE pod.
     *
     * WHAT THIS IS:
     *   - Measure FRONT/BACK distance from robot center
     *     to the strafe pod wheel contact point.
     *
     * WHY IT EXISTS:
     *   - When robot ROTATES, the strafe pod moves in an arc.
     *   - This offset lets us subtract rotation-induced motion.
     */
    public static final double STRAFE_OFFSET_IN = 0.0; // MEASURE THIS


    // ===========================================================
    // 4) INTERNAL CONVERSION CONSTANT
    // ===========================================================

    /**
     * TICKS_TO_INCHES
     * -----------------------------------------------------------
     * Converts encoder ticks into linear inches.
     */
    private static final double TICKS_TO_INCHES =
            (Math.PI * WHEEL_DIAMETER_IN) / TICKS_PER_REV;


    // ===========================================================
    // 5) HARDWARE
    // ===========================================================

    private final DcMotorEx forwardOdo;  // forward odometry encoder
    private final DcMotorEx strafeOdo;   // strafe odometry encoder
    private final BNO055IMU imu;         // Control Hub internal IMU


    // ===========================================================
    // 6) ROBOT POSE STATE
    // ===========================================================

    private double x = 0.0;        // Field X position (inches)
    private double y = 0.0;        // Field Y position (inches)
    private double heading = 0.0;  // Robot heading (radians)

    // Previous loop values (used to compute deltas)
    private int prevForward = 0;
    private int prevStrafe = 0;
    private double prevHeading = 0.0;


    // ===========================================================
    // 7) CONSTRUCTOR
    // ===========================================================

    /**
     * Initializes hardware and IMU.
     *
     * Device names required in Robot Configuration:
     *   - "forwardOdo"
     *   - "strafeOdo"
     *   - "imu"
     */
    public TwoPodOdometry(HardwareMap hw) {

        forwardOdo = hw.get(DcMotorEx.class, "forwardOdo");
        strafeOdo  = hw.get(DcMotorEx.class, "strafeOdo");

        // Reset encoders so starting position = 0
        forwardOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        strafeOdo.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Odometry wheels are NOT used for motor control
        forwardOdo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        strafeOdo.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU (gyro)
        imu = hw.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS; // REQUIRED
        imu.initialize(params);

        // Capture initial heading
        heading = getImuHeading();
        prevHeading = heading;
    }


    // ===========================================================
    // 8) UPDATE LOOP (CALL EVERY ITERATION)
    // ===========================================================

    /**
     * update()
     * -----------------------------------------------------------
     * Must be called EVERY loop in auto/teleop.
     *
     * This:
     *   - Reads encoders
     *   - Reads IMU heading
     *   - Computes movement since last loop
     *   - Updates field (x, y, heading)
     */
    public void update() {

        // --- 1) Get current heading from IMU
        heading = getImuHeading();

        // Change in heading since last update
        double dTheta = angleWrap(heading - prevHeading);
        prevHeading = heading;

        // --- 2) Read encoder positions
        int currForward = forwardOdo.getCurrentPosition() * FORWARD_DIR;
        int currStrafe  = strafeOdo.getCurrentPosition()  * STRAFE_DIR;

        // Encoder deltas
        int dFticks = currForward - prevForward;
        int dSticks = currStrafe  - prevStrafe;

        prevForward = currForward;
        prevStrafe  = currStrafe;

        // --- 3) Convert ticks → inches
        double dF = dFticks * TICKS_TO_INCHES;
        double dS = dSticks * TICKS_TO_INCHES;

        // --- 4) Remove rotation-induced motion
        // When the robot rotates, pods move in arcs.
        // These terms subtract that effect.
        double robotForward = dF - (FORWARD_OFFSET_IN * dTheta);
        double robotStrafe  = dS - (STRAFE_OFFSET_IN  * dTheta);

        // --- 5) Rotate robot motion into field coordinates
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        x += robotForward * cos - robotStrafe * sin;
        y += robotForward * sin + robotStrafe * cos;
    }


    // ===========================================================
    // 9) IMU HELPERS
    // ===========================================================

    private double getImuHeading() {
        Orientation angles = imu.getAngularOrientation();
        return angles.firstAngle;
    }

    /**
     * Keeps angle between -π and +π
     * Prevents sudden jumps at wraparound.
     */
    private double angleWrap(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a < -Math.PI) a += 2.0 * Math.PI;
        return a;
    }


    // ===========================================================
    // 10) POSE ACCESSORS
    // ===========================================================

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeading() { return heading; }

    /**
     * Manually set the robot's pose.
     * Useful at the start of autonomous.
     */
    public void setPose(double x, double y, double headingRad) {
        this.x = x;
        this.y = y;
        this.heading = headingRad;
        this.prevHeading = headingRad;
    }
}
