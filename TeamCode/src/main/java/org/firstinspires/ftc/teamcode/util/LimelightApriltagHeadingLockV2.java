package org.firstinspires.ftc.teamcode.util;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class LimelightApriltagHeadingLockV2 {
    public static double Kp = 0.02;
    public static double Ki = 0.0;
    public static double Kd = 0.002;
    public static double maxTurn = 0.6;
    public static double targetTx = 0.0; // center tag

//    private DcMotor leftFront = null;
//    private DcMotor rightFront = null;
//    private DcMotor leftBack = null;
//    private DcMotor rightBack = null;

    // Below ignore
    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime timer = new ElapsedTime();

    public LimelightApriltagHeadingLockV2(HardwareMap hwmap) {
//        leftFront = hwmap.get(DcMotor.class, "leftFront");
//        rightFront = hwmap.get(DcMotor.class, "rightFront");
//        leftBack = hwmap.get(DcMotor.class, "leftBack");
//        rightBack = hwmap.get(DcMotor.class, "rightBack");
//
//        // Setting motor directions so they don't make the bot go in circles
//        leftFront.setDirection(DcMotor.Direction.REVERSE);
//        rightFront.setDirection(DcMotor.Direction.FORWARD);
//        rightBack.setDirection(DcMotor.Direction.FORWARD);
//        leftBack.setDirection(DcMotor.Direction.FORWARD);
    }

    public double update(double tx) {
        double error = targetTx - tx;

        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-3;

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;

        double output =
                (Kp * error) +
                        (Ki * integralSum) +
                        (Kd * derivative);

        output = Range.clip(output, -maxTurn, maxTurn);

        lastError = error;
        timer.reset();
        return -output;
//        double d = Math.max(Math.abs(0) + Math.abs(0) + Math.abs(-output), 1);
//        drive(0, 0, -output, d, false);
    }

    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }

//    public void drive(double leftY, double leftX, double rightX, double d, boolean leftBump) {
//        // Calculate motor power
//        double lfp = leftY + leftX + rightX / d;
//        double rfp = leftY - leftX - rightX / d;
//        double lbp = leftY - leftX + rightX / d;
//        double rbp = leftY + leftX - rightX / d;
//
//        // Var is multiplied by the power amount input, so >1 means less power, and 1 means it stays the same
//        double powerReductionFactor = 1;
//        if (leftBump) {
//            powerReductionFactor = 0.5;
//        }
//
//        // Set motor power
//        leftFront.setPower(powerReductionFactor*lfp);
//        rightFront.setPower(powerReductionFactor*rfp);
//        leftBack.setPower(powerReductionFactor*lbp);
//        rightBack.setPower(powerReductionFactor*rbp);

        // Telemetry
//        telemetry.addData("rightFront", rightFront.getDirection());
//        telemetry.addData("leftFront", leftFront.getDirection());
//        telemetry.addData("rightBack", rightBack.getDirection());
//        telemetry.addData("leftBack", leftBack.getDirection());
//        telemetry.addData("Power Reduction Factor", powerReductionFactor);
//    }
}
