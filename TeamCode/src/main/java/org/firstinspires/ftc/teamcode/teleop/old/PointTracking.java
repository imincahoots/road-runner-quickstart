
package org.firstinspires.ftc.teamcode.teleop.old;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.misc.FieldCentricDrive;

@TeleOp
@Disabled
public class PointTracking extends OpMode {
    FieldCentricDrive drive;

    DcMotor feeder, feeder2;
    DcMotorEx launcher, launcher2;
    Servo servo;

    double launchSpeed = 0.0;
    double driveForward, driveRight, turn;
    boolean rBumpOld = false;
    boolean lBumpOld = false;
    ElapsedTime timer = new ElapsedTime();

    boolean headingLock = false;
    boolean toggleOld = false;
    double targetX = 0.0; // Change to your desired target
    double targetY = 0.0;
    double startX = 0.0;      // inches
    double startY = 0.0;      // inches
    double startHeading = 0.0; // radians (0 = facing forward)



    @SuppressLint("DefaultLocale")
    @Override
    public void init() {
        // TUNE these constants for your hardware:
        double TICKS_PER_REV = 8192.0;      // example: change to your encoder
        double ODO_WHEEL_DIAM = 1.0;        // inches
        double GEAR_RATIO = 1.0;            // odometry gearing
        double TRACK_WIDTH = 12.25;          // inches between left/right odometry wheels
        double LATERAL_OFFSET = 0.75;        // lateral wheel forward offset in inches

        drive = new FieldCentricDrive(hardwareMap,
                TICKS_PER_REV, ODO_WHEEL_DIAM, GEAR_RATIO,
                TRACK_WIDTH, LATERAL_OFFSET);

        feeder = hardwareMap.get(DcMotor.class, "feeder");
        feeder2 = hardwareMap.get(DcMotor.class, "feeder2");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        servo = hardwareMap.get(Servo.class, "servo");

        feeder.setDirection(DcMotor.Direction.FORWARD);
        feeder2.setDirection(DcMotor.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        drive.resetPose(startX, startY, startHeading);



    }

    @Override
    public void loop() {
        // Read sticks
        double leftStickY = gamepad1.left_stick_y; // typically -1 forward
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;


        boolean toggleNew = gamepad1.left_stick_button; // Press left stick to toggle
        if (toggleNew && !toggleOld) {
            headingLock = !headingLock;
        }
        toggleOld = toggleNew;


        // Map: left stick Y -> forward (invert so positive forward), left stick X -> right
        driveForward = -leftStickY;
        driveRight = leftStickX;
        if (headingLock) {
            double[] pose = drive.getPose(); // [x, y, heading]
            double dx = targetX - pose[0];
            double dy = targetY - pose[1];
            double desiredHeading = Math.atan2(dy, dx);
            double headingError = desiredHeading - pose[2];

            // Normalize error to [-π, π]
            headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

            // Simple proportional control
            turn = headingError * 1.5; // Tune gain as needed
        } else {
            turn = rightStickX;
        }


        // Update odometry first
        drive.update();
        double[] pose = drive.getPose(); // [x, y, heading]

        // Scale by left trigger for precision control (same behavior as before)
        double speedScale = 1.0 - gamepad1.left_trigger;
        drive.setDrivePower(driveForward * speedScale, driveRight * speedScale, turn * speedScale);

        // Launcher speed toggles (edge detection)
        boolean rBumpNew = gamepad1.right_bumper;
        if (rBumpNew && !rBumpOld) launchSpeed += 0.1;
        boolean lBumpNew = gamepad1.left_bumper;
        if (lBumpNew && !lBumpOld) launchSpeed -= 0.1;
        rBumpOld = rBumpNew;
        lBumpOld = lBumpNew;

        // Servo
        servo.setPosition(gamepad1.right_trigger > 0 ? 0.75 : 0.5);

        // Launcher & feeder control
        if (gamepad1.a) {
            launcher.setPower(-launchSpeed - 0.05);
            launcher2.setPower(launchSpeed);
        }
        if (gamepad1.b || gamepad2.left_bumper) {
            launcher.setPower(0);
            launcher2.setPower(0);
        }

        // feeder2 control
        if (gamepad1.x || gamepad2.x) {
            feeder2.setPower(-1);
        } else if (gamepad2.a) {
            feeder2.setPower(1);
        } else {
            feeder2.setPower(0);
        }

        // feeder control
        if (gamepad2.b) {
            feeder.setPower(1);
        } else if (gamepad1.y || gamepad2.y) {
            feeder.setPower(-1);
        } else {
            feeder.setPower(0);
        }

        // Telemetry from OpMode only
        telemetry.addData("Status", "Running");
        telemetry.addData("Launch Power", String.format("%.2f", launchSpeed));
        telemetry.addData("Drive Forward", String.format("%.2f", driveForward));
        telemetry.addData("Drive Right", String.format("%.2f", driveRight));
        telemetry.addData("Turn", String.format("%.2f", turn));
        telemetry.addData("Pose X (in)", String.format("%.2f", pose[0]));
        telemetry.addData("Pose Y (in)", String.format("%.2f", pose[1]));
        telemetry.addData("Heading (deg)", String.format("%.1f", Math.toDegrees(pose[2])));
        telemetry.addData("Heading Lock", headingLock ? "ON" : "OFF");
        telemetry.addData("Target Point", String.format("(%.1f, %.1f)", targetX, targetY));

        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }
}

