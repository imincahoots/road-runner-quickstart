package org.firstinspires.ftc.teamcode.auto.old;

import androidx.annotation.NonNull;

// Road Runner imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// FTC SDK imports
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
public class AutoActions {

    // ---------------------------
    // Dashboard Tunable Variables
    // ---------------------------
    public static double FEED_DURATION = .88;   // seconds
    public static double FEED_POWER = -1.0;      // (-1.0 to 1.0)

    public static double LAUNCH_DURATION = 1.5; // seconds
    public static double LAUNCH_POWER = 0.42;    // (-1.0 to 1.0)
    public static double CYCLE_DURATION = .5;
    public static double CYCLE_POWER = -1.0;
    // ---------------------------
// Dashboard Tunable Variables for LaunchTrigger
// ---------------------------
    public static double TRIGGER_POSITION = 0.75;  // Servo target position
    public static double TRIGGER_DURATION = 0.5;   // seconds to hold the position

    // ---------------------------
    // Subsystem: LaunchServo
// ---------------------------
    public static class LaunchServo {
        private final CRServo feed;
        private final CRServo feed2;

        public LaunchServo(HardwareMap hardwareMap) {
            feed = hardwareMap.get(CRServo.class, "feed");
            feed2 = hardwareMap.get(CRServo.class, "feed2");

        }

        public void setPower(double position) {
            feed.setPower(position);
            feed2.setPower(-position);
        }

    }


    // ---------------------------
    // Subsystem: Feeder
    // ---------------------------
    public static class Feeder {
        private final DcMotorEx feederMotor1;
        private final DcMotorEx feederMotor2;

        public Feeder(HardwareMap hardwareMap) {
            feederMotor1 = hardwareMap.get(DcMotorEx.class, "feeder");
            feederMotor2 = hardwareMap.get(DcMotorEx.class, "feeder2");
        }

        public void setPower(double power) {
            feederMotor1.setPower(-power);
            feederMotor2.setPower(power);
        }


        public void stop() {
            feederMotor1.setPower(0);
            feederMotor2.setPower(0);
        }
    }

    public static class AngleServos {
        public final Servo angleR;
        public final Servo angleL;

        public AngleServos(HardwareMap hardwareMap) {
            this(hardwareMap, "angleR", "angleL");
        }

        public AngleServos(HardwareMap hardwareMap, String angleRName, String angleLName) {
            angleR = hardwareMap.get(Servo.class, angleRName);
            angleL = hardwareMap.get(Servo.class, angleLName);
        }

        // direct setters similar to Launcher.setPower / setVelocity
        public void setPositions(double posR, double posL) {
            angleR.setPosition(posR);
            angleL.setPosition(posL);
        }

        // convenience: set angleL to opposite of posR (1 - posR)
        public void setOpposite(double posR) {
            angleR.setPosition(posR);
            angleL.setPosition(.6 - posR);
        }
    }



    // ---------------------------
    // Subsystem: Launcher
    // ---------------------------
    public static class Launcher {
        private final DcMotorEx launcherMotor1;
        private final DcMotorEx launcherMotor2;

        public Launcher(HardwareMap hardwareMap) {
            launcherMotor1 = hardwareMap.get(DcMotorEx.class, "launcher");
            launcherMotor2 = hardwareMap.get(DcMotorEx.class, "launcher2");
        }

        public void setPower(double power) {
            launcherMotor1.setPower(-power);
            launcherMotor2.setPower(power);
        }


        public void setVelocity(double velocity) {
            launcherMotor1.setVelocity(-velocity*2660);
            launcherMotor2.setVelocity(velocity*2660);
        }

        public void runUsingEncoder(){
            launcherMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcherMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public double getVelocity() {
            return (launcherMotor2.getVelocity());
        }

        public void stop() {
            launcherMotor1.setPower(0);
            launcherMotor2.setPower(0);
        }
    }

    // ---------------------------
    // Action: FeedIn (runs feeder motors for duration)
    // ---------------------------
    public static class FeedIn implements Action {
        private final Feeder feeder;
        private final double duration;  // seconds
        private final double power;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean initialized = false;

        public FeedIn(Feeder feeder, double duration, double power) {
            this.feeder = feeder;
            this.duration = duration;
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Initialization
            if (!initialized) {
                timer.reset();
                feeder.setPower(power);
                initialized = true;
            }

            double elapsed = timer.seconds();
            packet.put("Feeder elapsed", elapsed);
            packet.put("Feeder power", power);

            // Check if action is done
            if (elapsed >= duration) {
                feeder.stop();        // stop feeder
                initialized = false;  // reset initialized flag
                timer.reset();        // optional: reset timer for reuse
                return false;         // action finished
            }

            return true; // keep running
        }
    }
    public static class CycleR implements Action{
        private final LaunchServo launchServo;
        private final double power;
        private final double duration;
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();

        public CycleR(LaunchServo launchServo, double duration, double power){
            this.launchServo = launchServo;
            this.power = power;
            this.duration = duration;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            if (!initialized){
                launchServo.setPower(-power);
                timer.reset();
                initialized = true;
            }
            double elapsed = timer.seconds();
            if (elapsed >= duration){
                launchServo.setPower(0);
                initialized = false;
                return false;
            }
            return true;
        }
    }

    public static class CycleNext implements Action {
        private final Feeder feeder;
        private final double duration;  // seconds
        private final double power;
        private final ElapsedTime timer = new ElapsedTime();
        private boolean initialized = false;

        public CycleNext(Feeder feeder, double duration, double power) {
            this.feeder = feeder;
            this.duration = duration;
            this.power = power;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            // Initialization
            if (!initialized) {
                timer.reset();
                feeder.setPower(power);
                initialized = true;
            }

            double elapsed = timer.seconds();
            packet.put("Feeder elapsed", elapsed);
            packet.put("Feeder power", power);

            // Check if action is done
            if (elapsed >= duration) {
                feeder.stop();        // stop feeder
                initialized = false;  // reset initialized flag
                timer.reset();        // optional: reset timer for reuse
                return false;         // action finished
            }
            return true; // keep running
        }
    }

    /* Action that sets the two servos to opposite positions.
       Implements the non-generic com.acmerobotics.roadrunner.Action interface. */
    public static class ServoSetOppositeAction implements Action {
        private final AngleServos servos;
        private final double posR;
        private final double posL;
        private boolean started = false;

        // infer opposite for posL if caller supplies only posR
        public ServoSetOppositeAction(AngleServos servos, double posL) {
            this(servos, posL, 0.6 - posL);
        }

        public ServoSetOppositeAction(AngleServos servos, double posR, double posL) {
            this.servos = servos;
            this.posR = posR;
            this.posL = posL;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!started) {
                servos.setPositions(posR, posL);
                packet.put("angleR", posR);
                packet.put("angleL", posL);
                started = true;
            }

            // second call: finish and reset for reuse
            return false;
        }

        /**
         * Prepare this instance to be reused.
         */
        public void reset() {
            started = false;
        }

        /**
         * Cancel simply resets internal state.
         */
        public void cancel() {
            started = false;
        }
    }

        // ---------------------------
// Action: LaunchTrigger
// ---------------------------
        public static class LaunchTrigger implements Action {
            private final LaunchServo launchServo;
            private final double targetPosition;
            private final double duration;
            private final ElapsedTime timer = new ElapsedTime();
            private boolean initialized = false;

            public LaunchTrigger(LaunchServo launchServo, double targetPosition, double duration) {
                this.launchServo = launchServo;
                this.targetPosition = targetPosition;
                this.duration = duration;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    launchServo.setPower(targetPosition);
                    initialized = true;
                }

                double elapsed = timer.seconds();
                packet.put("LaunchTrigger elapsed", elapsed);
                packet.put("LaunchTrigger position", targetPosition);

                if (elapsed >= duration) {
                    // Optionally, you could reset servo here if needed
                    launchServo.setPower(0);
                    return false; // done
                }

                return true; // keep running
            }
        }


    public static class LaunchSpinThenTrigger implements Action {
        private final Launcher launcher;
        private final LaunchServo launchServo;

        private final double spinDuration;      // max time to wait for spinup (s)
        private final double spinPower;         // fraction (0..1)
        private final double triggerDuration;   // how long to hold trigger (s)
        private final double settleTime = 1.5; // extra time at speed before firing

        private final ElapsedTime timer = new ElapsedTime();
        private boolean initialized = false;
        private boolean triggerActivated = false;

        private static final double TRIGGER_FIRE = 1;
        private static final double TRIGGER_RESET = 0;

        public LaunchSpinThenTrigger(Launcher launcher,
                                     LaunchServo launchServo,
                                     double spinDuration,
                                     double spinPower,
                                     double triggerDuration) {
            this.launcher = launcher;
            this.launchServo = launchServo;
            this.spinDuration = spinDuration;
            this.spinPower = spinPower;
            this.triggerDuration = triggerDuration;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                launcher.runUsingEncoder();
                launchServo.setPower(TRIGGER_RESET);
                initialized = true;
                triggerActivated = false;
            }

            double target = spinPower;
            double actual = Math.abs(launcher.getVelocity());

            // Spin-up phase
            if (!triggerActivated) {
                launcher.setVelocity(target); // keep commanding target
                boolean veloOk = actual >= 0.95 * target;
                boolean timeoutOk = timer.seconds() >= spinDuration;
                if ((veloOk && timer.seconds() >= settleTime) || timeoutOk) {
                    launchServo.setPower(TRIGGER_FIRE);
                    triggerActivated = true;
                    timer.reset();
                    packet.put("event", "trigger fired");
                }
                return true;
            }

            // Firing phase
            double triggerElapsed = timer.seconds();
            packet.put("triggerElapsed", triggerElapsed);
            if (triggerElapsed >= triggerDuration) {
                launchServo.setPower(TRIGGER_RESET);
                initialized = false;
                triggerActivated = false;
                return false; // finished
            } else {
                launcher.setVelocity(target); // hold velocity steady while firing
                return true;
            }
        }

        public void reset() {
            launcher.setVelocity(0.0);
            launchServo.setPower(TRIGGER_RESET);
            initialized = false;
            triggerActivated = false;
            timer.reset();
        }

        public void cancel() {
            launcher.setVelocity(0.0);
            launchServo.setPower(TRIGGER_RESET);
            initialized = false;
            triggerActivated = false;
            timer.reset();
        }
    }
}