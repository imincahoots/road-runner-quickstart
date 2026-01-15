package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * Field-centric mecanum drive using three passive encoder odometry wheels.
 * No IMU, no RoadRunner, and NO telemetry dependency.
 *
 * odoLeft and odoRight are parallel to robot forward (left/right deadwheels).
 * odoCenter is perpendicular (lateral) deadwheel.
 *
 * All distances in inches, heading in radians.
 */
public class FieldCentricDrive {
    // Drive motors
    public DcMotor leftFront, leftBack, rightFront, rightBack;

    // Odometry encoders (dead wheels)
    public DcMotor odoLeft, odoRight, odoCenter;

    // Pose state
    private double poseX = 0.0;     // inches
    private double poseY = 0.0;     // inches
    private double heading = 0.0;   // radians, integrated from odometry

    // Last encoder readings
    private double lastLeftTicks = 0.0;
    private double lastRightTicks = 0.0;
    private double lastCenterTicks = 0.0;

    // Hardware/odometry constants
    private final double TICKS_PER_REV;        // encoder ticks per revolution
    private final double ODO_WHEEL_DIAMETER;  // inches
    private final double GEAR_RATIO;          // odometry gearing (encoder-to-wheel)
    private final double INCHES_PER_TICK;     // derived

    private final double TRACK_WIDTH;         // distance between left and right odometry wheels (inches)
    private final double LATERAL_DISTANCE;    // forward offset of lateral wheel from robot center (inches)


    /**
     * Constructor (no telemetry).
     * Replace motor names with your config names if necessary.
     */
    public FieldCentricDrive(HardwareMap hw,
                             double ticksPerRev, double odoWheelDiameter, double gearRatio,
                             double trackWidth, double lateralDistance) {
        // constants
        this.TICKS_PER_REV = ticksPerRev;
        this.ODO_WHEEL_DIAMETER = odoWheelDiameter;
        this.GEAR_RATIO = gearRatio;
        this.INCHES_PER_TICK = 0.00198584842;
        this.TRACK_WIDTH = trackWidth;
        this.LATERAL_DISTANCE = lateralDistance;

        // map drive motors (change names to match your config)
        leftFront  = hw.dcMotor.get("leftFront");
        leftBack   = hw.dcMotor.get("leftBackPar");
        rightFront = hw.dcMotor.get("rightFrontPerp");
        rightBack  = hw.dcMotor.get("rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // map odometry encoders (names must match robot config)
        odoLeft = hw.dcMotor.get("feeder");
        odoRight = hw.dcMotor.get("feeder2");
        odoCenter = hw.dcMotor.get("rightFrontPerp");

        odoLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        odoRight.setDirection(DcMotorSimple.Direction.REVERSE);
        odoCenter.setDirection(DcMotorSimple.Direction.REVERSE);



        // configure encoders (treated as encoders only)
        odoLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        odoCenter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set directions if your encoders read inverted counts (adjust as necessary)
        odoLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        odoRight.setDirection(DcMotorSimple.Direction.FORWARD);
        odoCenter.setDirection(DcMotorSimple.Direction.FORWARD);

        // initialize last tick values
        lastLeftTicks = odoLeft.getCurrentPosition();
        lastRightTicks = odoRight.getCurrentPosition();
        lastCenterTicks = odoCenter.getCurrentPosition();
    }

    /**
     * Call periodically (loop) to integrate odometry and update pose.
     */
    public void update() {
        double curLeft = odoLeft.getCurrentPosition();
        double curRight = odoRight.getCurrentPosition();
        double curCenter = odoCenter.getCurrentPosition();

        double dLeftTicks = curLeft - lastLeftTicks;
        double dRightTicks = curRight - lastRightTicks;
        double dCenterTicks = curCenter - lastCenterTicks;

        lastLeftTicks = curLeft;
        lastRightTicks = curRight;
        lastCenterTicks = curCenter;

        double dLeftIn = dLeftTicks * INCHES_PER_TICK;
        double dRightIn = dRightTicks * INCHES_PER_TICK;
        double dCenterIn = dCenterTicks * INCHES_PER_TICK;

        // delta heading from left/right encoder difference
        double dTheta = (dRightIn - dLeftIn) / TRACK_WIDTH; // radians

        // update integrated heading
        double prevHeading = heading;
        heading += dTheta;
        heading = normalizeRadians(heading);

        // forward displacement (robot frame)
        double dForward = (dLeftIn + dRightIn) / 2.0;
        // lateral displacement corrected for lateral wheel offset
        double dLateral = dCenterIn - dTheta * LATERAL_DISTANCE;

        double dxRobot, dyRobot;
        if (Math.abs(dTheta) < 1e-6) {
            dxRobot = dForward;
            dyRobot = dLateral;
        } else {
            double rForward = dForward / dTheta;
            double rLateral = dLateral / dTheta;
            dxRobot = rForward * Math.sin(dTheta) + rLateral * (1 - Math.cos(dTheta));
            dyRobot = rForward * (1 - Math.cos(dTheta)) + rLateral * Math.sin(dTheta);
        }

        // rotate robot-relative delta into field coordinates using mid-heading approximation
        double midHeading = prevHeading + dTheta / 2.0;
        double cosH = Math.cos(midHeading);
        double sinH = Math.sin(midHeading);

        double dxField = dxRobot * cosH - dyRobot * sinH;
        double dyField = dxRobot * sinH + dyRobot * cosH;

        poseX += dxField;
        poseY += dyField;
    }

    /**
     * Field-centric drive.
     * desiredFieldForward: positive forward along field, desiredFieldRight: positive right, turn: rotation.
     */
    public void setDrivePower(double desiredFieldForward, double desiredFieldRight, double turn) {
        // convert field vector into robot-relative using current heading
        double cosH = Math.cos(-heading);
        double sinH = Math.sin(-heading);

        double robotForward = desiredFieldForward * cosH - desiredFieldRight * sinH;
        double robotLateral = desiredFieldForward * sinH + desiredFieldRight * cosH;

        // mecanum mixing (robotForward: forward, robotLateral: right, turn: rotation)
        double lf = robotForward + robotLateral + turn;
        double rf = robotForward - robotLateral - turn;
        double lb = robotForward - robotLateral + turn;
        double rb = robotForward + robotLateral - turn;

        // normalize
        double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max; rf /= max; lb /= max; rb /= max;
        }

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftBack.setPower(lb);
        rightBack.setPower(rb);
    }

    public double[] getPose() {
        return new double[]{poseX, poseY, heading};
    }

    public void resetPose(double x, double y, double headingRad) {
        this.poseX = x;
        this.poseY = y;
        this.heading = normalizeRadians(headingRad);
        lastLeftTicks = odoLeft.getCurrentPosition();
        lastRightTicks = odoRight.getCurrentPosition();
        lastCenterTicks = odoCenter.getCurrentPosition();
    }

    public void stop() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }

    private double normalizeRadians(double a) {
        while (a > Math.PI) a -= 2.0 * Math.PI;
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        return a;
    }
}
