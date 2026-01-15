package org.firstinspires.ftc.teamcode.misc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Config
@TeleOp(name = "Launcher Velocity Dashboard Test")
public class LauncherVelocityDashboardTest extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx launcher1 = hardwareMap.get(DcMotorEx.class, "launcher");
        DcMotorEx launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");

        launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        FtcDashboard dashboard = FtcDashboard.getInstance();


        waitForStart();

        while (opModeIsActive()) {

           launcher1.setVelocity(.45 * 2020);
           telemetry.addData("vel", launcher1.getVelocity());
           telemetry.update();
        }

        launcher1.setPower(0);
        launcher2.setPower(0);
    }
}
