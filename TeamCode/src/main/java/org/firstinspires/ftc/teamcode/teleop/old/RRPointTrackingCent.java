package org.firstinspires.ftc.teamcode.teleop.old;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.misc.MecanumDrive;

@TeleOp
public class RRPointTrackingCent extends OpMode {
    MecanumDrive drive;

    DcMotor feeder, feeder2;
    DcMotorEx launcher, launcher2;

    // Added from SOURCE
    Servo angleR;
    Servo angleL;
    CRServo feed;
    CRServo feed2;
    double angleServo = 0.0;
    boolean RBumpOld1;
    boolean LBumpOld1;
    boolean RBumpOld2;
    boolean LBumpOld2;

    double launchSpeed = 0.0;
    double driveForward, driveRight, turn;
    boolean rBumpOld = false;
    boolean lBumpOld = false;
    ElapsedTime timer = new ElapsedTime();

    boolean headingLock = false;
    boolean toggleOld = false;
    double targetX = -65.0; // inches on field
    double targetY = 65.0; // inches on field

    // alternate target point (used by toggle)
    double alternateTargetX = -65.0;
    double alternateTargetY = -65.0;
    boolean alternateMode = false;
    boolean optionYOld = false;

    // starting pose (set values as needed)
    double startX = 0;
    double startY = 0;
    double startHeading = Math.PI; // radians

    private DistanceToAngleSpeed mapper;

    @SuppressLint("DefaultLocale")
    @Override
    public void init() {

        mapper = new DistanceToAngleSpeed();

        // instantiate your Roadrunner-based MecanumDrive with a start pose
        Pose2d startPose = new Pose2d(startX, startY, startHeading);
        drive = new MecanumDrive(hardwareMap, startPose);

        // hardware preserved from your previous OpMode
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        feeder2 = hardwareMap.get(DcMotor.class, "feeder2");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        feed = hardwareMap.get(CRServo.class, "feed");
        feed2 = hardwareMap.get(CRServo.class, "feed2");


        // angle servos from SOURCE
        angleR = hardwareMap.get(Servo.class, "angleR");
        angleL = hardwareMap.get(Servo.class, "angleL");

        feeder.setDirection(DcMotor.Direction.FORWARD);
        feeder2.setDirection(DcMotor.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        // ensure MecanumDrive localizer pose is set to the chosen start pose
        drive.localizer.setPose(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update Roadrunner localizer and get robot pose & velocity
        // updatePoseEstimate() returns PoseVelocity2d and also advances the localizer internally
        PoseVelocity2d robotVel = drive.updatePoseEstimate(); // returned velocity (not used directly)
        Pose2d pose = drive.localizer.getPose(); // current pose from your localizer


        // Read joystick inputs
        double leftStickY = gamepad1.left_stick_y; // typically -1 forward
        double leftStickX = gamepad1.left_stick_x;
        double rightStickX = gamepad1.right_stick_x;

        // Toggle heading lock (edge detect)
        boolean toggleNew = gamepad1.right_stick_button;
        if (toggleNew && !toggleOld) {
            headingLock = !headingLock;
        }
        toggleOld = toggleNew;

        if (gamepad1.dpad_down){
            drive.localizer.setPose(new Pose2d(0, 0, Math.PI));
        }

        // Edge-detect Options + A to toggle alternate target
        boolean optionYNew = gamepad1.options && gamepad1.y;
        if (optionYNew && !optionYOld) {
            alternateMode = !alternateMode;
            if (alternateMode) {
                targetX = alternateTargetX;
                targetY = alternateTargetY;
            } else {
                targetX = -65.0;
                targetY = 65.0;
            }
        }
        optionYOld = optionYNew;

        // Map sticks to field-forward/field-right (we'll send these raw to drive)
        driveForward = -leftStickY;
        driveRight = -leftStickX;

        // distance to current target (in inches)
        double dxToTarget = targetX - pose.position.x;
        double dyToTarget = targetY - pose.position.y;
        double distToTarget = Math.hypot(dxToTarget, dyToTarget);
        telemetry.addData("Dist to Target (in)", String.format("%.2f", distToTarget));

        DistanceToAngleSpeed.Output out = mapper.getInterpolated(distToTarget);

        if (headingLock) {
            // compute desired heading toward target point using current pose
            double dx = targetX - pose.position.x;
            double dy = targetY - pose.position.y;
            double desiredHeading = Math.atan2(dy, dx);

            double headingError = desiredHeading - pose.heading.toDouble();
            // normalize to [-pi, pi]
            headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

            // proportional control (no clamping)
            double kP = 1.1; // tune as needed
            turn = headingError * kP;


        } else {
            turn = -rightStickX;
        }

        // Apply speed scaling using left trigger (same as your previous behavior)
        double speedScale = 1.0 - gamepad1.left_trigger;

        // Build a simple PoseVelocity2d containing requested velocities:
        // PoseVelocity2d(fields: forward, lateral/right, angular) consistent with MecanumDrive.setDrivePowers usage
        PoseVelocity2d powers = new PoseVelocity2d(new Vector2d(driveForward * speedScale, driveRight * speedScale),
                turn * speedScale);


        // Send to drive. MecanumDrive.setDrivePowers expects PoseVelocity2d
        drive.setDrivePowers(powers);


        // Launcher speed toggles (edge detection))
        boolean rBumpNew = gamepad1.right_bumper;
        if (rBumpNew && !rBumpOld) launchSpeed += 0.02;
        boolean lBumpNew = gamepad1.left_bumper;
        if (lBumpNew && !lBumpOld) launchSpeed -= 0.02;
        rBumpOld = rBumpNew;
        lBumpOld = lBumpNew;


        // ---------- gamepad2 bumpers control angle servos ----------
       /** boolean rBumpNew2 = gamepad2.right_bumper;
        boolean lBumpNew2 = gamepad2.left_bumper;

        final double STEP = 0.1;
        if (rBumpNew2 && !RBumpOld2) {
            angleServo += STEP;
        }
        if (lBumpNew2 && !LBumpOld2) {
            angleServo -= STEP;
        }

        RBumpOld2 = rBumpNew2;
        LBumpOld2 = lBumpNew2;

        // clamp to valid servo range
        if (angleServo < 0.0) angleServo = 0.0;
        if (angleServo > 0.6) angleServo = 0.6;

        // set servos with mirrored mapping
        angleL.setPosition(angleServo);
        angleR.setPosition(0.6 - angleServo);**/


        // ---------- Servo control ----------
        if (gamepad1.right_trigger >= 1){
            feed.setPower(1);
            feed2.setPower(-1);
        } else{
            feed.setPower(0);
            feed2.setPower(0);
        }
        //launcher.setVelocity(-(launchSpeed)*2660);
        //launcher2.setVelocity(launchSpeed*2660);

        // Launcher & feeder control
        if (gamepad1.b) {
            launchSpeed = 0;
        }

        // feeder2 control
        if (gamepad1.x || gamepad2.x) {
            feeder2.setPower(1);
        } else if (gamepad2.a) {
            feeder2.setPower(-1);
        } else {
            feeder2.setPower(0);
        }

        // feeder control
        if (gamepad2.b) {
            feeder.setPower(-1);
        } else if (gamepad1.y || gamepad2.y) {
            feeder.setPower(1);
        } else {
            feeder.setPower(0);
        }
        if (gamepad2.right_bumper){
            feed.setPower(-1);
            feed2.setPower(1);
        }
        angleL.setPosition(out.angle);
        angleR.setPosition(0.6 - out.angle);
        launcher.setVelocity(-(out.speed-0.0+launchSpeed-.05)*2660);
        launcher2.setVelocity((out.speed-0.0+launchSpeed-.05)*2660);



        // Telemetry
        telemetry.addData("Status", "Running");
        telemetry.addData("Launch Power", String.format("%.2f", launchSpeed));
        telemetry.addData("Drive Forward", String.format("%.2f", driveForward));
        telemetry.addData("Drive Right", String.format("%.2f", driveRight));
        telemetry.addData("Turn", String.format("%.2f", turn));
        telemetry.addData("Heading (deg)", String.format("%.1f", Math.toDegrees(pose.heading.toDouble())));
        telemetry.addData("Heading Lock", headingLock ? "ON" : "OFF");
        telemetry.addData("Alternate Mode", alternateMode ? "ON" : "OFF");
        // current robot position (always)
        telemetry.addData("Robot Pos", String.format("(%.2f, %.2f)", pose.position.x, pose.position.y));

        // current active target location
        telemetry.addData("Current Target", String.format("(%.1f, %.1f)", targetX, targetY));

        // start coordinates (configured start pose)
        telemetry.addData("Start Pose", String.format("(%.1f, %.1f, %.2f°)", startX, startY, Math.toDegrees(startHeading)));

        // SOURCE angle telemetry
        telemetry.addData("angle", angleServo);
        telemetry.addData("angleServo", "%.3f", angleServo);
        telemetry.addData("angleL cmd", "%.3f", angleServo);
        telemetry.addData("angleR cmd", "%.3f", .6 - angleServo);
        telemetry.addData("Interp Angle", "%.3f", out.angle);
        telemetry.addData("Interp Speed", "%.3f", out.speed);
        telemetry.update();


        telemetry.update();
    }

    @Override
    public void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
    }
}