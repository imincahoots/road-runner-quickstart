package org.firstinspires.ftc.teamcode.misc;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class AngleChangeTest extends OpMode {
    MecanumDrive drive;

    // -------------------------
    // Motors and Servos
    // -------------------------
    DcMotor leftBackPar, leftFront, rightBack, rightFrontPerp;
    DcMotor feeder, feeder2;
    DcMotorEx launcher, launcher2;
    Servo servo;
    Servo angleR;
    Servo angleL;

    // -------------------------
    // Control variables
    // -------------------------
    double launchSpeed = 0;
    boolean leftBump, rightBump;
    double drivePower, turn, lateral;
    double rightStickX, rightStickY, leftStickX, leftStickY;

    // Set up toggle button variables
    boolean rBumpOld;

    boolean lBumpOld;
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
        servo = hardwareMap.get(Servo.class, "servo");
        angleR = hardwareMap.get(Servo.class, "angleR");
        angleL = hardwareMap.get(Servo.class, "angleL");

        // Set motor behaviors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontPerp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackPar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBackPar.setDirection(DcMotor.Direction.REVERSE);
        rightFrontPerp.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        feeder.setDirection(DcMotor.Direction.FORWARD);
        feeder2.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
    }

    // -------------------------
    // Main control loop
    // -------------------------
    @Override
    public void loop() {
    if (gamepad1.a){
        angleL.setPosition(0);
        angleR.setPosition(0);
    }
    if (gamepad1.b){
        angleR.setPosition(1);
        angleL.setPosition(-1);
    }
    if (gamepad1.x){
        angleR.setPosition(-1);
        angleL.setPosition(1);
    }

    }
}

    // ---------- Helper method ----------