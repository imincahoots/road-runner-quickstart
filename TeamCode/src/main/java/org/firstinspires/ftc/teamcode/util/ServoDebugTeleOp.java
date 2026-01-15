package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Servo Debug (Dashboard)", group = "Debug")
public class ServoDebugTeleOp extends OpMode {
    // ===== Dashboard-controlled servo positions =====
    public static double blockerLPos = 0.0;
    public static double blockerRPos = 0.0;
    public static double kickerLPos  = 0.0;
    public static double kickerRPos  = 0.0;
    public static double anglePos    = 0.0;

    // ===== Servos =====
    private Servo blockerL;
    private Servo blockerR;
    private Servo kickerL;
    private Servo kickerR;
    private Servo angle;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        blockerL = hardwareMap.get(Servo.class, "blockerL");
        blockerR = hardwareMap.get(Servo.class, "blockerR");
        kickerL  = hardwareMap.get(Servo.class, "kickerL");
        kickerR  = hardwareMap.get(Servo.class, "kickerR");
        angle    = hardwareMap.get(Servo.class, "angle");

        telemetry.addLine("Servo Debug TeleOp Initialized");
        telemetry.addLine("Use FTC Dashboard → Config to change values");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Clamp values just in case
        blockerL.setPosition(clamp(blockerLPos));
        blockerR.setPosition(clamp(blockerRPos));
        kickerL.setPosition(clamp(kickerLPos));
        kickerR.setPosition(clamp(kickerRPos));
        angle.setPosition(clamp(anglePos));

        telemetry.addData("Blocker L", blockerLPos);
        telemetry.addData("Blocker R", blockerRPos);
        telemetry.addData("Kicker L", kickerLPos);
        telemetry.addData("Kicker R", kickerRPos);
        telemetry.addData("Angle", anglePos);
        telemetry.update();
    }

    private double clamp(double val) {
        return Math.max(0.0, Math.min(1.0, val));
    }
}
