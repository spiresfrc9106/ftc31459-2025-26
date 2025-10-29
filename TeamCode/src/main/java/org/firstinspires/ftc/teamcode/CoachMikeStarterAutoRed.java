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

    @Override
    public void runOpMode() throws InterruptedException {

        int level=0;

        // Tom and Sammy: You really wanted the y 24 to be -24
        Pose2d initialPose = new Pose2d(new Vector2d(68,12), Math.toRadians(180));

        TankDrive drive = new TankDrive(hardwareMap, initialPose);

        UserCommands wheelSpinUp = ()->{return false;};

        UserCommands wheelSpinDown = ()->{return false;};
        UserCommands commandLaunch = ()->{return false;};

        CoachMikeStarterShooter shooter = new CoachMikeStarterShooter(
                hardwareMap, wheelSpinDown, wheelSpinUp, commandLaunch
        );


        // Wait for the game to start (driver presses START)
        waitForStart();

        VelConstraint endVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(10),
                        new AngularVelConstraint(5)
                ));

        AccelConstraint endAccelConstraint = new ProfileAccelConstraint(-5, 10);

        Action actionDrive = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(40, 12), Math.toRadians(180), endVelConstraint, endAccelConstraint)
                .build();

        Action actionsWhileFirstDrive = new SequentialAction(
                new ParallelAction(actionDrive, shooter.new SpinUpAutonomous()),
                shooter.new LaunchAutonomous(),
                shooter.new SpinDownAutonomous());

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
