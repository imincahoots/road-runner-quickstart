package org.firstinspires.ftc.teamcode.auto.old;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.old.AutoActions.AngleServos;
import org.firstinspires.ftc.teamcode.auto.old.AutoActions.CycleNext;
import org.firstinspires.ftc.teamcode.auto.old.AutoActions.CycleR;
import org.firstinspires.ftc.teamcode.auto.old.AutoActions.FeedIn;
import org.firstinspires.ftc.teamcode.auto.old.AutoActions.Feeder;
import org.firstinspires.ftc.teamcode.auto.old.AutoActions.LaunchServo;
import org.firstinspires.ftc.teamcode.auto.old.AutoActions.LaunchSpinThenTrigger;
import org.firstinspires.ftc.teamcode.auto.old.AutoActions.Launcher;
import org.firstinspires.ftc.teamcode.auto.old.AutoActions.ServoSetOppositeAction;
import org.firstinspires.ftc.teamcode.misc.MecanumDrive;

@Config
@Autonomous(name = "auto far", group = "Autonomous")
public class AutoFar extends LinearOpMode {

    // If you want to keep an action field (optional)
    // private Action launchAction;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems using hardwareMap
        Feeder feeder = new Feeder(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);
        LaunchServo launchServo = new LaunchServo(hardwareMap);
        Pose2d initialPose = new Pose2d(0, 0, Math.PI);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AngleServos angleServos = new AngleServos(hardwareMap);

        // Paths
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(-30);



        telemetry.addData("Feed Duration", AutoActions.FEED_DURATION);
        telemetry.addData("Feed Power", AutoActions.FEED_POWER);
        telemetry.addData("Launch Duration", AutoActions.LAUNCH_DURATION);
        telemetry.addData("Launch Power", AutoActions.LAUNCH_POWER);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Create fresh LaunchSpinThenTrigger instances when used.
        Action launchClose1 = new LaunchSpinThenTrigger(launcher, launchServo, 5, .44, .3);
        Action launchClose2 = new LaunchSpinThenTrigger(launcher, launchServo,5,  .44, .3);


        Action launchFar = new LaunchSpinThenTrigger(
                launcher,
                launchServo,
                5,
                .45,
                .3
        );
        Action feedAction = new FeedIn(
                feeder,
                AutoActions.FEED_DURATION, // seconds
                AutoActions.FEED_POWER     // power (0..1)
        );
        Action feedAction2 = new FeedIn(
                feeder,
                .5, // seconds
                AutoActions.FEED_POWER     // power (0..1)
        );
        Action cycleR = new CycleR(
                launchServo,
                .2,
                1
        );
        Action cycle = new CycleNext(feeder, AutoActions.CYCLE_DURATION, AutoActions.CYCLE_POWER);
        Action setAngles = new ServoSetOppositeAction(angleServos, 0.3);
        Action setAnglesFar = new ServoSetOppositeAction(angleServos, 0.3);
        telemetry.addData("Velocity", launcher.getVelocity());
        telemetry.update();






        // Run the planned sequence once (Actions.runBlocking will call run() repeatedly)
        Actions.runBlocking(
                new SequentialAction(
                tab1.build()

                )
        );

        // Stop all motors after sequence
        feeder.stop();
        launcher.stop();
        telemetry.update();
    }
}