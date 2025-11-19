package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public final class SparkyJrShooter {
    public static class Params {
        public double FEED_TIME_SECONDS = 0.5; //The feeder servos run this long when a shot is requested.

        public double WAIT_AFTER_BACKWARDS_TIME_SECONDS = 1.0;

        public double FEED_PAUSE_TIME_SECONDS = 0.2; //Pause this long after a Feed before feeding again.
        public double SERVO_STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
        public double SERVO_FULL_SPEED = 1.0;

        /*
         * When we control our launcher motor, we are using encoders. These allow the control system
         * to read the current speed of the motor and apply more or less power to keep it at a constant
         * velocity. Here we are setting the target, and minimum velocity that the launcher should run
         * at. The minimum velocity is a threshold for determining when to fire.
         */

        public enum LauncherSpeed {
            FAR(0),
            MEDIUM(1),
            CLOSE(2);

            public final int index;

            LauncherSpeed(int index) {
                this.index = index;
            }
        }

        public double[] LAUNCHER_TARGET_VELOCITY_RPS = {65.0, 60.0, 55.0};
        public double[] LAUNCHER_MIN_VELOCITY_RPS = {64.5, 59.5, 54.5};
    }

    public static Params PARAMS = new Params();

    public double LAUNCHER_STOP_VELOCITY_RPS = 0;

    public double TICKS_PER_REVOLUTION = 24;

    // Declare OpMode members.

    private DcMotorEx launcherMotor = null;
    private CRServo leftFeederServo = null;
    private CRServo rightFeederServo = null;

    ElapsedTime feederTimer = new ElapsedTime();
    private Params.LauncherSpeed currentSpeed = Params.LauncherSpeed.FAR;

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        // SPIN_BACK,
        // WAIT_AFTER_SPIN_BACK,
        LAUNCH,
        LAUNCHING,
        AFTER_LAUNCH_PAUSE,
    }

    private LaunchState launchState;

    private UserCommands commandWheelSpinDown;
    private UserCommands commandWheelSpinUpFar;
    private UserCommands commandWheelSpinUpMedium;
    private UserCommands commandWheelSpinUpClose;
    private UserCommands commandLaunching;
    private UserCommands commandWheelSpinBack;


    public SparkyJrShooter(
            HardwareMap hardwareMap,
            UserCommands commandWheelSpinDown,
            UserCommands commandWheelSpinUpFar,
            UserCommands commandWheelSpinUpMedium,
            UserCommands commandWheelSpinUpClose,
            UserCommands commandLaunching,
            UserCommands commandWheelSpinBack
    ) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */

        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeederServo = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeederServo = hardwareMap.get(CRServo.class, "right_feeder");


        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        launcherMotor.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeederServo.setPower(PARAMS.SERVO_STOP_SPEED);
        rightFeederServo.setPower(PARAMS.SERVO_STOP_SPEED);

        launcherMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeederServo.setDirection(DcMotorSimple.Direction.REVERSE);


        this.commandWheelSpinDown = commandWheelSpinDown;
        this.commandWheelSpinUpFar = commandWheelSpinUpFar;
        this.commandWheelSpinUpMedium = commandWheelSpinUpMedium;
        this.commandWheelSpinUpClose = commandWheelSpinUpClose;
        this.commandLaunching = commandLaunching;
        this.commandWheelSpinBack = commandWheelSpinBack;

    }


    public void sendPlotData(@NonNull TelemetryPacket p) {
        p.put("launchState", launchState);
        p.put("flyWheelSpeed RPS", launchMotorGetVelocityRPS());

        p.put("targetFlyWheelSpeed RPS", PARAMS.LAUNCHER_TARGET_VELOCITY_RPS[currentSpeed.index]);
        p.put("speedMode", currentSpeed); // Logs "FAR", "MEDIUM", or "CLOSE"
    }


    public double launchMotorGetVelocityRPS() {
        double velocityTicksPerSecond = launcherMotor.getVelocity();
        double curVelocityRPS = velocityTicksPerSecond / TICKS_PER_REVOLUTION;
        return curVelocityRPS;
    }

    public void launchMotorSetVelocityRPS(double velocityRPS){
        double velocityTicksPerSecond = velocityRPS * TICKS_PER_REVOLUTION;
        launcherMotor.setVelocity(velocityTicksPerSecond);
    }

    boolean launch(boolean shotRequested) {
        boolean launchCompleted = false;
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launchMotorSetVelocityRPS(PARAMS.LAUNCHER_TARGET_VELOCITY_RPS[currentSpeed.index]);

                if (launchMotorGetVelocityRPS() > PARAMS.LAUNCHER_MIN_VELOCITY_RPS[currentSpeed.index]) {
                    launchState = LaunchState.LAUNCH;
                    feederTimer.reset();
                    leftFeederServo.setPower(-PARAMS.SERVO_FULL_SPEED);
                    rightFeederServo.setPower(-PARAMS.SERVO_FULL_SPEED);
                }
                break;
            /*case SPIN_BACK:
                if (feederTimer.seconds() > PARAMS.FEED_TIME_SECONDS) {
                    launchState = LaunchState.WAIT_AFTER_SPIN_BACK;
                    leftFeederServo.setPower(PARAMS.SERVO_STOP_SPEED);
                    rightFeederServo.setPower(PARAMS.SERVO_STOP_SPEED);
                    feederTimer.reset();
                }
                break;
            case WAIT_AFTER_SPIN_BACK:
                if (feederTimer.seconds() > PARAMS.WAIT_AFTER_BACKWARDS_TIME_SECONDS) {
                    launchState = LaunchState.LAUNCH;
                }*/
            case LAUNCH:
                leftFeederServo.setPower(PARAMS.SERVO_FULL_SPEED);
                rightFeederServo.setPower(PARAMS.SERVO_FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > PARAMS.FEED_TIME_SECONDS) {
                    launchState = LaunchState.AFTER_LAUNCH_PAUSE;
                    leftFeederServo.setPower(PARAMS.SERVO_STOP_SPEED);
                    rightFeederServo.setPower(PARAMS.SERVO_STOP_SPEED);
                }
                break;
            case AFTER_LAUNCH_PAUSE:
                if (feederTimer.seconds() > PARAMS.FEED_TIME_SECONDS + PARAMS.FEED_PAUSE_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    launchCompleted = true;
                }
        }
        return launchCompleted;
    }

    public class LaunchTeleOp implements Action {

        public LaunchTeleOp() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            /*
             * Here we give the user control of the speed of the launcher motor without automatically
             * queuing a shot.
             */
            if (commandWheelSpinUpFar.isCommanded()) {
                currentSpeed = Params.LauncherSpeed.FAR;
                launchMotorSetVelocityRPS(PARAMS.LAUNCHER_TARGET_VELOCITY_RPS[currentSpeed.index]);
            } else if (commandWheelSpinUpMedium.isCommanded()) {
                currentSpeed = Params.LauncherSpeed.MEDIUM;
                launchMotorSetVelocityRPS(PARAMS.LAUNCHER_TARGET_VELOCITY_RPS[currentSpeed.index]);
            } else if (commandWheelSpinUpClose.isCommanded()) {
                currentSpeed = Params.LauncherSpeed.CLOSE;
                launchMotorSetVelocityRPS(PARAMS.LAUNCHER_TARGET_VELOCITY_RPS[currentSpeed.index]);
            } else if (commandWheelSpinDown.isCommanded()) { // stop flywheel
                launchMotorSetVelocityRPS(LAUNCHER_STOP_VELOCITY_RPS);
            }

            /*
             * Now we call our "Launch" function.
             */
            if(!commandWheelSpinBack.isCommanded()) {
                launch(commandLaunching.isCommanded());
            }
            sendPlotData(telemetryPacket);

            return true; // a return true means that run should run again.
        }
    }

    public class LaunchAutonomous implements Action {
        private final Params.LauncherSpeed autoSpeed;

        public LaunchAutonomous(Params.LauncherSpeed speed) {
            this.autoSpeed = speed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            currentSpeed = autoSpeed;
            boolean launchCompleted = launch(true);

            sendPlotData(telemetryPacket);

            return !launchCompleted; // a return true means that run should run again.
        }
    }

    public class SpinUpAutonomous implements Action {
        private final Params.LauncherSpeed autoSpeed;

        public SpinUpAutonomous(Params.LauncherSpeed speed) {
            this.autoSpeed = speed;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            currentSpeed = autoSpeed;
            launchMotorSetVelocityRPS(PARAMS.LAUNCHER_TARGET_VELOCITY_RPS[currentSpeed.index]);

            sendPlotData(telemetryPacket);

            return false; // a return true means that run should run again.
        }
    }

    public class SpinDownAutonomous implements Action {

        public SpinDownAutonomous() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            launchMotorSetVelocityRPS(LAUNCHER_STOP_VELOCITY_RPS);

            sendPlotData(telemetryPacket);

            return false; // a return true means that run should run again.
        }
    }
}
