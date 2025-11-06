package org.firstinspires.ftc.teamcode;

// import static org.firstinspires.ftc.teamcode.RobotConstants.state;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

@Autonomous(name="MikeStarterAutoRed", group="Robot", preselectTeleOp = "MikeStarterTeleOp")
@Config
public class CoachMikeStarterAutoRed extends LinearOpMode {

    public static double startNow = 1.0;
    public static double moveFactor = 0.5;
    public static double rotationFactor = 0.5;
    public static double moveAccelFactor = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        int level=0;

        //Pose2d initialPose = new Pose2d(new Vector2d(45,-53), Math.toRadians(-1));
        Pose2d initialPose = new Pose2d(new Vector2d(53, -51), Math.toRadians(114));
        Pose2d secondPose = new Pose2d(new Vector2d(41, -39), Math.toRadians(120));
        Pose2d thirdPose = new Pose2d(new Vector2d(-12, 24), Math.toRadians(135));
        Pose2d fourthPose = new Pose2d(new Vector2d( -30, 53), Math.toRadians(135));

        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        UserCommands wheelSpinUp = ()->{return false;};

        UserCommands wheelSpinDown = ()->{return false;};
        UserCommands commandLaunch = ()->{return false;};

        CoachMikeStarterShooter shooter = new CoachMikeStarterShooter(
                hardwareMap, wheelSpinDown, wheelSpinUp, commandLaunch
        );


        // Wait for the game to start (driver presses START)
        waitForStart();

        VelConstraint velConstraint =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(drive.PARAMS.maxWheelVel*moveFactor),
                        new AngularVelConstraint(drive.PARAMS.maxAngVel*rotationFactor)
                ));

        AccelConstraint accelConstraint = new ProfileAccelConstraint(drive.PARAMS.minProfileAccel*moveAccelFactor, drive.PARAMS.maxProfileAccel*moveAccelFactor);


        Action actionDrive = drive.actionBuilder(initialPose)
                .splineTo(secondPose.position,secondPose.heading, velConstraint, accelConstraint)
                //.splineTo(new Vector2d(destX, destY), Math.toRadians(135), endVelConstraint, endAccelConstraint)
                //.turn(Math.toRadians(-44), new TurnConstraints(15,-15, 15))
                .build();

        Action actionsWhileFirstDrive = new SequentialAction(
                new ParallelAction(actionDrive, shooter.new SpinUpAutonomous()),
                new SleepAction(1.0),
                shooter.new LaunchAutonomous(),
                new SleepAction(1.0),
                shooter.new LaunchAutonomous(),
                new SleepAction(1.0),
                shooter.new LaunchAutonomous(),
                shooter.new SpinDownAutonomous(),
                drive.actionBuilder(secondPose)
                        .splineTo(thirdPose.position, thirdPose.heading, velConstraint, accelConstraint)
                        .splineTo(fourthPose.position, fourthPose.heading, velConstraint, accelConstraint)
                        .build()
        );

        List<Action> runningActions = new ArrayList<>();
        runningActions.add(actionsWhileFirstDrive);

        List<Action> doActions = new ArrayList<>();

        boolean start = false;

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();
            if (!start && startNow>0.0) {
                start = true;
                doActions = runningActions;
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
}
