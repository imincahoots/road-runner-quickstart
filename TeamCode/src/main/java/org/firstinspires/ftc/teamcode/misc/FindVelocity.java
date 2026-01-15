package org.firstinspires.ftc.teamcode.misc;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@TeleOp
public class FindVelocity extends OpMode {
    DcMotorEx launcher;
    DcMotorEx launcher2;
    double velocity1;
    double velocity2;
    @Override
    @SuppressLint("DefaultLocale")
    public void init() {
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

    }
    @Override
    public void loop() {
        launcher.setPower(1);
        launcher2.setPower(-1);
        velocity1 = launcher.getVelocity();
        velocity2 = launcher2.getVelocity();

        telemetry.addData("velocity1", velocity1);
        telemetry.addData("velocity2", velocity2);
        telemetry.update();
    }
}
