package org.firstinspires.ftc.teamcode;

// import static org.firstinspires.ftc.teamcode.RobotConstants.state;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="AutoBlue", group="Robot", preselectTeleOp = "MecanumTeleOpV3")
@Config
public class MecanumAutoV3c extends LinearOpMode {

    public static double startNow = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        int level=0;

        // Tom and Sammy: You really wanted the y 24 to be -24
        Pose2d initialPose = new Pose2d(new Vector2d(68,-12), Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ElapsedTime timer1 = new ElapsedTime();

        // Wait for the game to start (driver presses START)
        waitForStart();

        VelConstraint endVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(10),
                        new AngularVelConstraint(5)
                ));

        AccelConstraint endAccelConstraint = new ProfileAccelConstraint(-5, 10);

        Action a = drive.actionBuilder(initialPose)
                // Tom and Sammy: this is in initialPose:.splineTo(new Vector2d(-68,24), Math.toRadians(0))
                // Tom and Sammy: You really wanted y=-12
                .splineTo(new Vector2d(40, -12), Math.toRadians(180), endVelConstraint, endAccelConstraint)
                // Tom and Sammy, removing the Vel and Accel constraints: .splineTo(new Vector2d(12, 12), Math.toRadians(135), endVelConstraint, endAccelConstraint )
                // Tom and Sammy, can't splineTo where we are, use .turn: .splineTo(new Vector2d(12, 12), Math.toRadians(90), endVelConstraint, endAccelConstraint )
               // .turn(Math.toRadians(-45), new TurnConstraints(15,-15, 15)) // This is a relative angle
                .build();

        List<Action> runningActions = new ArrayList<>();
        runningActions.add(a);

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