package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.misc.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.LimelightApriltagHeadingLockV2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
@TeleOp(name = "BoggV1", group = "TeleOp")
public class BoggV1 extends OpMode {
    public static boolean onBotDebugTelemetry = false;

    boolean a = false, b = false, x = false, y = false, leftBump = false, rightBump = false, up = false, down = false, left = false, right = false, leftStickButton = false, rightStickButton = false;
    double rightTrig = 0, leftTrig = 0, leftStickY = 0, leftStickX = 0, rightStickY = 0, rightStickX = 0;
    boolean a2 = false, b2 = false, x2 = false, y2 = false, leftBump2 = false, rightBump2 = false, up2 = false, down2 = false, left2 = false, right2 = false, leftStickButton2 = false, rightStickButton2 = false;
    double rightTrig2 = 0, leftTrig2 = 0, leftStickY2 = 0, leftStickX2 = 0, rightStickY2 = 0, rightStickX2 = 0;
    boolean rBumpOld = false, lBumpOld = false, upOld = false, downOld = false, lastRightTrigState = false, lastLeft = false, lastRight = false, spinFlywheel = false, lastRightStickButton = false, isHeadingLocking = false, flywheelOn = false;
    double drivePowerMultiplyFactor = 1.00;
    String teleopStatus = "";
    DcMotorEx flywheel = null;
    DcMotor feeder = null, intake = null;
    Servo blockerR = null, blockerL = null, angle = null, kickerR = null, kickerL = null;
    Pose2d startPose = null;
    MecanumDrive drive = null;
    ElapsedTime loopTimer = new ElapsedTime();
    private Limelight3A limelight;
    private IMU imu;
    private LimelightApriltagHeadingLockV2 tagLock;

    public static double startX = 0;
    public static double startY = 0;
    public double targetDist = 0;
    public static double startHeading = Math.toRadians(180);
    public static double flywheelTargetVelocity = 20;
    public static double standbyVelocity = 0;
    public static int rumbleDuration = 100;
    public static int velocityBumpAmount = 20;
    public static double driveSpeedMin = 0.20;
    public static double flywheelP = 300;
    public static double flywheelI = 1;
    public static double flywheelD = 4;
    public static double flywheelF = 10;
    public static double rightBlockerMax = 0;
    public static double rightBlockerMin = 0.5;
    public static double leftBlockerMax = 1;
    public static double leftBlockerMin = 0.5;
    public static double rightKickerMax = 0.8;
    public static double rightKickerMin = 0.5;
    public static double leftKickerMax = 0.8;
    public static double leftKickerMin = 0.5;
    public static double distToTarget = 0;
    public static double angleMax = 0.6;
    public static double angleMin = 0.9;
    public static double angleBumpAmount = 0.1;
    private DistanceToAngleSpeed mapper;
    @Override
    public void init() {
         telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

         flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
         flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         flywheel.setPIDFCoefficients(
                 DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(flywheelP, flywheelI, flywheelD, flywheelF)
         );
         flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
         feeder = hardwareMap.get(DcMotorEx.class, "feeder");
         intake = hardwareMap.get(DcMotorEx.class, "intake");

         blockerR = hardwareMap.get(Servo.class, "blockerR");
         blockerL = hardwareMap.get(Servo.class, "blockerL");
         kickerR = hardwareMap.get(Servo.class, "kickerR");
         kickerL = hardwareMap.get(Servo.class, "kickerL");
         angle = hardwareMap.get(Servo.class, "angle");

         startPose = new Pose2d(startX, startY, startHeading);
         drive = new MecanumDrive(hardwareMap, startPose);
         drive.localizer.setPose(startPose);

         limelight = hardwareMap.get(Limelight3A.class, "limelight");
         limelight.pipelineSwitch(0); // Set pipeline to 1 - for the training bot
         imu = hardwareMap.get(IMU.class, "imu");
         // Set hub orientation because the hub has a imu built in
         RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP);
         imu.initialize(new IMU.Parameters(revHubOrientationOnRobot)); // Give the imu the starting orientation
         tagLock = new LimelightApriltagHeadingLockV2(hardwareMap);

        mapper = new DistanceToAngleSpeed();

         telemetry.addData("Status", "Initialized");
         telemetry.update();
    }

    @Override
    public void start() {
        limelight.start();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        loopTimer.reset();
        // typically -1 forward
        leftStickY = gamepad1.left_stick_y;
        leftStickX = gamepad1.left_stick_x;
        rightStickY = gamepad1.right_stick_y;
        rightStickX = gamepad1.right_stick_x;
        leftStickButton = gamepad1.left_stick_button;
        rightStickButton = gamepad1.right_stick_button;
        leftBump = gamepad1.left_bumper;
        rightBump = gamepad1.right_bumper;
        leftTrig = gamepad1.left_trigger;
        rightTrig = gamepad1.right_trigger;
        a = gamepad1.a;
        b = gamepad1.b;
        x = gamepad1.x;
        y = gamepad1.y;
        up = gamepad1.dpad_up;
        down = gamepad1.dpad_down;
        left = gamepad1.dpad_left;
        right = gamepad1.dpad_right;

        leftStickY2 = gamepad2.left_stick_y;
        leftStickX2 = gamepad2.left_stick_x;
        rightStickY2 = gamepad2.right_stick_y;
        rightStickX2 = gamepad2.right_stick_x;
        leftStickButton2 = gamepad2.left_stick_button;
        rightStickButton2 = gamepad2.right_stick_button;
        leftBump2 = gamepad2.left_bumper;
        rightBump2 = gamepad2.right_bumper;
        leftTrig2 = gamepad2.left_trigger;
        rightTrig2 = gamepad2.right_trigger;
        a2 = gamepad2.a;
        b2 = gamepad2.b;
        x2 = gamepad2.x;
        y2 = gamepad2.y;
        up2 = gamepad2.dpad_up;
        down2 = gamepad2.dpad_down;
        left2 = gamepad2.dpad_left;
        right2 = gamepad2.dpad_right;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());


        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            Pose3D botpose1 = result.getBotpose();     // Megatag1 pose set
            Pose3D botpose2 = result.getBotpose_MT2(); // Megatag2 pose set


            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        }


        // Velocity changing (gamepad1)
        boolean rBumpNew = gamepad1.right_bumper;
        if (rBumpNew && !rBumpOld) flywheelTargetVelocity += velocityBumpAmount;
        boolean lBumpNew = gamepad1.left_bumper;
        if (lBumpNew && !lBumpOld) flywheelTargetVelocity -= velocityBumpAmount;
        rBumpOld = rBumpNew;
        lBumpOld = lBumpNew;

        // Angle changing (gamepad1)
        boolean upNew = gamepad1.dpad_up;
        boolean downNew = gamepad1.dpad_down;
        double currentAngle = angle.getPosition();

        if (upNew && !upOld) {
            // Bump angle DOWN
            currentAngle -= angleBumpAmount;
            teleopStatus = "up";
        }
        if (downNew && !downOld) {
            // Bump angle UP
            currentAngle += angleBumpAmount;
            teleopStatus = "down";
        }

        currentAngle = Math.min(angleMax, Math.max(angleMin, currentAngle));
        angle.setPosition(currentAngle);
        upOld = upNew;
        downOld = downNew;

        // Controller feedback for when target velocity (gamepad1)
        if ((flywheel.getVelocity() < flywheelTargetVelocity * 1.05) && (flywheel.getVelocity() > flywheelTargetVelocity * 0.95)) {
            gamepad1.rumble(rumbleDuration);
        } else {
            gamepad1.stopRumble();
        }

        // Flywheel trigger starter
//        boolean rightTrigPress = gamepad1.right_trigger >= flywheelTriggerSens;
//        if (rightTrigPress && !lastRightTrigState) {
//            spinFlywheel = !spinFlywheel;
//        }
//        lastRightTrigState = rightTrigPress;
//        if (spinFlywheel){
//            flywheel.setVelocity(flywheelTargetVelocity);
//        } else {
//            flywheel.setVelocity(standbyVelocity);
//        }
        if (result.isValid()){
            distToTarget = result.getBotposeAvgDist();
            telemetry.addData("Dist to Target (in)", String.format("%.2f", distToTarget));
            telemetry.addData("Dist to Target (ll)", String.format("%.2f", result.getBotposeAvgDist()));
            DistanceToAngleSpeed.Output out = mapper.getInterpolated(distToTarget);
            angle.setPosition(out.angle);
//            flywheelTargetVelocity = out.speed;
            }
        // Drive slowdown (gamepad1)
        if(leftTrig == 0) {
            drivePowerMultiplyFactor = 1;
        } else if (leftTrig == 1) {
            drivePowerMultiplyFactor = driveSpeedMin;
        } else {
            drivePowerMultiplyFactor = Math.abs(leftTrig - 1);
        }

        // Feeder + Intake (gamepad1)
        if(x) {
            // Intake
            feeder.setPower(1);
            intake.setPower(-1);
        } else {
            feeder.setPower(0);
            intake.setPower(0);
        }
        if(y) {
            // Outake
            feeder.setPower(-1);
            intake.setPower(1);
        } else {
            feeder.setPower(0);
            intake.setPower(0);
        }

        // Feeder + Intake (gamepad2)
        if(x2) {
            feeder.setPower(1);
            intake.setPower(-1);
        } else {
            feeder.setPower(0);
            intake.setPower(0);
        }
        if(y2) {
            feeder.setPower(-1);
            intake.setPower(1);
        } else {
            feeder.setPower(0);
            intake.setPower(0);
        }

        // Flywheel toggle (gamepad1 dpad right)
        if (right && !lastRight) {
            flywheelOn = !flywheelOn; // toggle state
        }

        lastRight = right;

        if (flywheelOn) {
            flywheel.setVelocity(flywheelTargetVelocity);
        } else {
            flywheel.setVelocity(standbyVelocity);
        }

        // Blocker + kicker (gamepad2)
        if(leftTrig2 != 0) {
            blockerL.setPosition(leftBlockerMin);
            if(leftTrig2 == 1) {
                kickerL.setPosition(leftKickerMax);
            } else {
                kickerL.setPosition(leftKickerMin);
            }
        } else {
            blockerL.setPosition(leftBlockerMax);
        }

        if(rightTrig2 != 0) {
            blockerR.setPosition(rightBlockerMin);
            if(rightTrig2 == 1) {
                kickerR.setPosition(rightKickerMax);
            } else {
                kickerR.setPosition(rightKickerMin);
            }
        } else {
            blockerR.setPosition(rightBlockerMax);
        }

        // April tag heading lock
        double turn = rightStickX;
        if (rightStickButton && !lastRightStickButton) {
            isHeadingLocking = !isHeadingLocking; // Flip the state when button is first pressed
        }
        lastRightStickButton = rightStickButton;

        if (isHeadingLocking) {
            turn += tagLock.update(result.getTx());
            teleopStatus = "Locking using Limelight (ON)";
        } else {
            teleopStatus = "Not locking (OFF)";
        }
        // Debug
        if(leftStickButton && down) {
            imu.resetYaw();
        }
        if(leftStickButton && up) {
            //todo set imu orientation to limelight (recalibrate MT2 using MT1 heading/roadrunner heading)
        }
        /*

        have back left trigger slow the robot down like on elliot - done
        flywheels spin whole match
        x is intake 1st
        y is feeder
        right bump is increase velocity - done
        left bump is decrease velocity - done
        right stick button is heading lock (limelight) - done

        a is outtake 1

        b is outtake feeder

        2nd driver

        left trigger move left blocker at trigger .5 position then trigger at position 1
        right trigger move right blocker at trigger .5 position then trigger at position 1
        a outtake 1
        b outtake feeder
        x intake 1
        y feeder

        (Intake 2 = feeder)

        Fixed controls below
        gamepad1:

        leftStickButton = gamepad1.left_stick_button; =
        rightStickButton = gamepad1.right_stick_button; = heading lock
        leftBump = gamepad1.left_bumper; =  flywheel speed down
        rightBump = gamepad1.right_bumper; =  flywheel speed up
        leftTrig = gamepad1.left_trigger; = robot speed slow down
        rightTrig = gamepad1.right_trigger; =

        irrevelant letter pads, ignore. A intake X outake
        a = gamepad1.a; = outtake front wheels
        b = gamepad1.b; = outtake feeder wheels
        x = gamepad1.x; = intake front wheels
        y = gamepad1.y; = outtake feeder wheels
        up = gamepad1.dpad_up; = bump angle changer up
        down = gamepad1.dpad_down; = bump angle changer down
        left = gamepad1.dpad_left; = fetch shooting settings and apply
        right = gamepad1.dpad_right; = flywheel toggle

        gamepad2:

        leftStickButton = gamepad2.left_stick_button; =
        rightStickButton = gamepad2.right_stick_button; =
        leftBump = gamepad2.left_bumper; =
        rightBump = gamepad2.right_bumper; =
        basically press down trigger at all blocker goes down, then when it is all the way down kicker goes up and ball goes weeeee
        leftTrig = gamepad2.left_trigger; = move left blocker if trigger is >0, move left kicker servo  if trigger is < 1
        rightTrig = gamepad2.right_trigger; = move right blocker if trigger is >0, move right kicker servo  if trigger is < 1
        a = gamepad2.a; = outtake front wheels
        b = gamepad2.b; = outtake feeder wheels
        x = gamepad2.x; = intake front wheels
        y = gamepad2.y; = intake feeder wheels
        up = gamepad2.dpad_up; =
        down = gamepad2.dpad_down; =
        left = gamepad2.dpad_left; =
        right = gamepad2.dpad_right; =
        */

        // I have it set so that the turning power isn't limited, easy change to add
        PoseVelocity2d powers = new PoseVelocity2d(new Vector2d(-leftStickY*drivePowerMultiplyFactor, -leftStickX*drivePowerMultiplyFactor), -turn);
        drive.setDrivePowers(powers);

        telemetry.addData("Status", teleopStatus);
        telemetry.addData("Flywheel Velocity", flywheel.getVelocity());
        telemetry.addData("Flywheel Target", flywheelTargetVelocity);
        telemetry.addLine();
        telemetry.addData("Bot Position RR", String.format("(%.2f, %.2f)", drive.localizer.getPose().position.x, drive.localizer.getPose().position.y));
        if(result.isValid()) {
            telemetry.addData("Bot Position Lime MT1", result.getBotpose());
            telemetry.addData("Bot Position Lime MT2", result.getBotpose_MT2());
        } else {
            telemetry.addData("Bot Position Lime MT1", "Unknown");
            telemetry.addData("Bot Position Lime MT2", "Unknown");
        }
        telemetry.addData("Heading RR", drive.localizer.getPose().heading); // Radians
        telemetry.addData("Heading IMU", orientation.getYaw()); // I think degrees?
        if(onBotDebugTelemetry) {
            telemetry.addData("Angle Servo", angle.getPosition());
            telemetry.addData("Left Blocker Servo", blockerL.getPosition());
            telemetry.addData("Right Blocker Servo", blockerR.getPosition());
            telemetry.addData("Left Kicker Servo", kickerL.getPosition());
            telemetry.addData("Right Kicker Servo", kickerR.getPosition());
            telemetry.addData("Drive Multiply Factor", drivePowerMultiplyFactor);
            telemetry.addData("Turn Lime Heading", turn);
            telemetry.addData("Loop Time Ms", loopTimer.milliseconds());
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
    }
}
