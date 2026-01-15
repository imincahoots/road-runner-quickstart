package org.firstinspires.ftc.teamcode.teleop.old;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.misc.MecanumDrive;

@TeleOp

public class RobotCodeClean extends OpMode {
    MecanumDrive drive;

    // -------------------------
    // Motors and Servos
    // -------------------------
    DcMotor leftBackPar, leftFront, rightBack, rightFrontPerp;
    DcMotor feeder, feeder2;
    DcMotorEx launcher, launcher2;
    CRServo feed;
    CRServo feed2;

    // -------------------------
    // Control variables
    // -------------------------
    double launchSpeed = 0;
    double angleServo = 0;
    boolean leftBump, rightBump;
    double drivePower, turn, lateral;
    double rightStickX, rightStickY, leftStickX, leftStickY;

    // Set up toggle button variables
    boolean RBumpOld1;

    boolean LBumpOld1;

    boolean RBumpOld2;
    boolean LBumpOld2;
    Servo angleR;
    Servo angleL;
    ElapsedTime timer = new ElapsedTime();

    enum State {IDLE, INTAKING, LAUNCHING, STOPPING}

    State state = State.IDLE;


    @SuppressLint("DefaultLocale")
    @Override
    public void init() {


        // Map hardware
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontPerp = hardwareMap.get(DcMotor.class, "rightFrontPerp");
        leftBackPar = hardwareMap.get(DcMotor.class, "leftBackPar");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        feeder = hardwareMap.get(DcMotor.class, "feeder");
        feeder2 = hardwareMap.get(DcMotor.class, "feeder2");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        feed = hardwareMap.get(CRServo.class, "feed");
        angleR = hardwareMap.get(Servo.class, "angleR");
        angleL = hardwareMap.get(Servo.class, "angleL");
        feed2 = hardwareMap.get(CRServo.class, "feed2");

        // Set motor behaviors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontPerp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackPar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set motor directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontPerp.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackPar.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        feeder.setDirection(DcMotor.Direction.FORWARD);
        feeder2.setDirection(DcMotor.Direction.REVERSE);



        telemetry.addData("Status", "Initialized");
        telemetry.addData("launch power", launchSpeed);
    }

    // -------------------------
    // Main control loop
    // -------------------------
    @Override
    public void loop() {

        // ---------- Read gamepad ----------
        leftBump = gamepad1.left_bumper;
        rightBump = gamepad1.right_bumper;
        rightStickX = gamepad1.right_stick_x;
        rightStickY = gamepad1.right_stick_y;
        leftStickX = gamepad1.left_stick_x;
        leftStickY = gamepad1.left_stick_y;

        drivePower = -leftStickY;
        lateral = leftStickX;
        turn = rightStickX;

        // ---------- Drive ----------
        double leftFrontPower = (drivePower + lateral + turn);
        double rightFrontPower = (drivePower - lateral - turn);
        double leftBackPower = (drivePower - lateral + turn);
        double rightBackPower = (drivePower + lateral - turn);

        leftFront.setPower(leftFrontPower * (1 - gamepad1.left_trigger));
        leftBackPar.setPower(leftBackPower * (1 - gamepad1.left_trigger));
        rightFrontPerp.setPower(rightFrontPower * (1 - gamepad1.left_trigger));
        rightBack.setPower(rightBackPower * (1 - gamepad1.left_trigger));

        // ---------- Adjust launcher speed ----------
        boolean rBumpNew = gamepad1.right_bumper;
        if (rBumpNew && !RBumpOld1) {
            launchSpeed += .05;
        }
        boolean lBumpNew = gamepad1.left_bumper;
        RBumpOld1 = rBumpNew;
        if (lBumpNew && !LBumpOld1) {
            launchSpeed -= .05;
        }
        LBumpOld1 = lBumpNew;

        // read current bumper states
        boolean rBumpNew2 = gamepad2.right_bumper;
        boolean lBumpNew2 = gamepad2.left_bumper;

// edge detection and change
        final double STEP = 0.05; // smaller step for smoother control
        if (rBumpNew2 && !RBumpOld2) {
            angleServo += STEP;
        }
        if (lBumpNew2 && !LBumpOld2) {
            angleServo -= STEP;
        }

// update old states AFTER processing edges
        RBumpOld2 = rBumpNew2;
        LBumpOld2 = lBumpNew2;

// clamp to valid servo range
        if (angleServo < 0.0) angleServo = 0.0;
        if (angleServo > 0.6) angleServo = 0.6;

// set servos with mirrored mapping
        angleL.setPosition(angleServo);
        angleR.setPosition(0.6 - angleServo);


        // ---------- Servo control ----------
            if (gamepad1.right_trigger >= 0.5){
                feed.setPower(1);
                feed2.setPower(-1);
            } else{
                feed.setPower(0);
                feed2.setPower(0);
            }

        // ---------- Launcher & Feeder control ----------
        controlLauncherAndFeeder();

        launcher.setVelocity(-(launchSpeed)*2660);
        launcher2.setVelocity(launchSpeed*2660);

        telemetry.addData("Status", "Running");
        telemetry.addData("Launch Power", launchSpeed);
        telemetry.addData("Drive Power", drivePower);
        telemetry.addData("Turn", turn);
        telemetry.addData("Lateral", lateral);
        telemetry.addData("angle", angleServo);
        telemetry.addData("angleServo", "%.3f", angleServo);
        telemetry.addData("angleL cmd", "%.3f", angleServo);
        telemetry.addData("angleR cmd", "%.3f", .6 - angleServo);
        telemetry.update();
        ;
    }

    // ---------- Helper method ----------
    private void controlLauncherAndFeeder() {
        // Compute desired powers (default 0)
        double desiredFeeder = 0.0;
        double desiredFeeder2 = 0.0;

        // feeder (DcMotor feeder)
        if (gamepad1.y || gamepad2.y) {
            desiredFeeder = 1.0;
        } else if (gamepad2.b) {
            desiredFeeder = -1.0;
        }

        // feeder2 (DcMotor feeder2)
        if (gamepad1.x || gamepad2.x) {
            desiredFeeder2 = 1.0;
        } else if (gamepad2.a) {
            desiredFeeder2 = -1.0;
        }

        // Apply once (avoid multiple overwrites)
        feeder.setPower(desiredFeeder);
        feeder2.setPower(desiredFeeder2);

        // Launcher stop button
        if (gamepad1.b) {
            launchSpeed = 0;
        }
    }

}
