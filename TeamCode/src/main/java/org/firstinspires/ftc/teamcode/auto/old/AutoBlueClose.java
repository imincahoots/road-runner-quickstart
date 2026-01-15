package org.firstinspires.ftc.teamcode.auto.old;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
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
@Autonomous(name = "auto blu close", group = "Autonomous")
public class AutoBlueClose extends LinearOpMode {

    // If you want to keep an action field (optional)
    // private Action launchAction;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize subsystems using hardwareMap
        Feeder feeder = new Feeder(hardwareMap);
        Launcher launcher = new Launcher(hardwareMap);
        LaunchServo launchServo = new LaunchServo(hardwareMap);
        Pose2d initialPose = new Pose2d(-60, -37, -Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        AngleServos angleServos = new AngleServos(hardwareMap);

        // Paths
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-24, -24), 3 * -(Math.PI / 4));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(-24, -24, 3 * -(Math.PI / 4)))
                .strafeToLinearHeading(new Vector2d(-12, -24), Math.PI / 2);

        TrajectoryActionBuilder tab25 = drive.actionBuilder(new Pose2d(-12, -24, Math.PI / 2))
                .strafeTo(new Vector2d(-11, -44));

        TrajectoryActionBuilder tab26 = drive.actionBuilder(new Pose2d(-11, -44, Math.PI / 2))
                .strafeToLinearHeading(new Vector2d(-12, -20), -Math.toRadians(125));
        TrajectoryActionBuilder tab27 = drive.actionBuilder(new Pose2d(-12, -20, -Math.toRadians(125)))
                .strafeToLinearHeading(new Vector2d(10, -30), Math.PI/2);
        TrajectoryActionBuilder tab28 = drive.actionBuilder(new Pose2d(10, -30, Math.PI/2))
                .strafeTo(new Vector2d(10, -44));
        TrajectoryActionBuilder tab29 = drive.actionBuilder(new Pose2d(10, -44, -Math.PI/2))
                .strafeToLinearHeading(new Vector2d(-12, -20), -Math.toRadians(140));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-12, -20, -Math.toRadians(140)))
                .strafeToLinearHeading(new Vector2d(-12, -24), -Math.toRadians(140));


        telemetry.addData("Feed Duration", AutoActions.FEED_DURATION);
        telemetry.addData("Feed Power", AutoActions.FEED_POWER);
        telemetry.addData("Launch Duration", AutoActions.LAUNCH_DURATION);
        telemetry.addData("Launch Power", AutoActions.LAUNCH_POWER);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Create fresh LaunchSpinThenTrigger instances when used.
        Action launchClose1 = new LaunchSpinThenTrigger(launcher, launchServo, 5, .42, .3);
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
                        tab1.build(),
                        setAngles,
                        launchClose1,
                        cycle,
                        launchClose2,
                        tab2.build(),
                        new ParallelAction(
                           tab25.build(),
                                feedAction
                        ),
                        cycleR,
                        setAnglesFar,
                        tab26.build(),
                        launchFar,
                        cycle,
                        launchFar,
                        tab27.build(),
                        new ParallelAction(
                                tab28.build(),
                                feedAction2
                        ),
                        cycleR,
                        tab29.build(),
                        launchFar,
                        cycle,
                        launchFar

                )
        );

        // Stop all motors after sequence
        feeder.stop();
        launcher.stop();
        telemetry.update();
    }
}