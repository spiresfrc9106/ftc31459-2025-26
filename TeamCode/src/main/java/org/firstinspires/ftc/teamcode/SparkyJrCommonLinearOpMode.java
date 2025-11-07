package org.firstinspires.ftc.teamcode;

// import static org.firstinspires.ftc.teamcode.RobotConstants.state;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

@Config
public class SparkyJrCommonLinearOpMode extends LinearOpMode {

    public static double startNow = 1.0;

    public List<Action> runningActionsList = new ArrayList<Action>();

    TankDrive drive = null;
    SparkyJrShooter shooter = null;

    TelemetryPacket packet = null;

    public boolean isTeleOp = false;

    public void intitialize(Pose2d initialPose) throws InterruptedException {
        drive = new TankDrive(hardwareMap, initialPose);

        UserCommands wheelSpinUp;

        UserCommands wheelSpinDown;
        UserCommands commandLaunch;

        if (isTeleOp) {
            wheelSpinUp = ()->{return gamepad1.y;};
            wheelSpinDown = ()->{return gamepad1.b;};
            commandLaunch = ()->{return gamepad1.leftBumperWasPressed();};
        } else {
            wheelSpinUp = ()->{return false;};
            wheelSpinDown = ()->{return false;};
            commandLaunch = ()->{return false;};
        }

        shooter = new SparkyJrShooter(
                hardwareMap, wheelSpinDown, wheelSpinUp, commandLaunch
        );

    }

    public void intitializeAuton(Pose2d initialPose) throws InterruptedException {
        isTeleOp = false;
        intitialize(initialPose);
    }
    public void intitializeTeleOp(Pose2d initialPose) throws InterruptedException {
        isTeleOp = true;
        intitialize(initialPose);
    }

    public void teleOpBody() throws InterruptedException {
        double xySpeedFactor = 0.25;
        double rotationSpeedFactor = 1.0;

        double x_vel_frac = gamepad1.left_stick_y;  // removed the minus sign to make
                                                    // the intake side the front in teleop
        double rot_vel_frac = -gamepad1.right_stick_x;

        boolean fullSpeed = gamepad1.right_bumper;
        if (fullSpeed) {
            xySpeedFactor = 1.0;
            rotationSpeedFactor = 4.0;
        }

        if (Math.abs(x_vel_frac)<0.05) {
            x_vel_frac = 0;
        }

        if (Math.abs(rot_vel_frac)<0.05) {
            rot_vel_frac = 0;
        }

        double xSpeedIPS = x_vel_frac * drive.PARAMS.maxWheelVel * xySpeedFactor;
        double rotSpeedRadPS = rot_vel_frac * drive.PARAMS.maxAngVel * rotationSpeedFactor;

        packet.put("x IPS", xSpeedIPS);
        packet.put("y IPS", xSpeedIPS);

        packet.put("rot DPS", Math.toDegrees(rotSpeedRadPS));

        drive.setRobotRelativeDrive(xSpeedIPS,rotSpeedRadPS);

        // Update everything. Odometry. Etc.
        drive.localizer.update();

        Canvas c = drive.sendPlotData(packet);
    }

    public void loopBody() throws InterruptedException {
        boolean start = false;
        List<Action> doActions = new ArrayList<>();

        // Wait for the game to start (driver presses START)
        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {
            packet = new TelemetryPacket();
            if (!start && startNow>0.0) {
                start = true;
                doActions = runningActionsList;
            }

            if (start && isTeleOp) {
                teleOpBody();
            }

            List<Action> newActions = new ArrayList<>();
            if (doActions.isEmpty()) {
                Canvas c = drive.sendPlotData(packet);
            }
            for (Action action : doActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            doActions = newActions;

            packet.put("Runtime (s)", getRuntime());

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }


    }

    @Override
    public void runOpMode() throws InterruptedException {



        SparkyJrAutonActionsFactory actionsFactory = new SparkyJrAutonActionsFactory(
                true
        );

        intitialize(actionsFactory.initialPose);

        runningActionsList = new ArrayList<Action>();

        Action runningAction = actionsFactory.buildAction(drive, shooter, SparkyJrAutonActionsFactory.StopPose.STOP_AT_POSE2);

        runningActionsList.add(runningAction);

        loopBody();

    }
}
