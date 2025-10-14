package org.firstinspires.ftc.teamcode;

// import static org.firstinspires.ftc.teamcode.RobotConstants.state;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;



@TeleOp(name="Robot: ExampleTeleOpV1", group="Robot")
@Config
public class ExampleTeleOpV1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        int level=0;

        Pose2d initialPose = new Pose2d(new Vector2d(-68,-24), Math.toRadians(0));

        TankDrive drive = new TankDrive(hardwareMap, initialPose);
        UserCommandsIntake userCommandIntake = ()->{return gamepad2.a;};
        UserCommandsPushOutOfIntake userCommandsPushOutOfIntake = ()->{return gamepad2.b;};
        ExampleIntake intake = new ExampleIntake(hardwareMap, userCommandIntake, userCommandsPushOutOfIntake);
        ElapsedTime timer1 = new ElapsedTime();

        // Wait for the game to start (driver presses START)
        waitForStart();

        List<Action> runningActions = new ArrayList<>();
        runningActions.add(intake.new PerhapsMoveIntakeTeleOp());

        List<Action> doActions = new ArrayList<>();

        boolean start = false;



        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();

            doActions = runningActions;

            List<Action> newActions = new ArrayList<>();
            if (doActions.isEmpty()) {
                drive.sendPlotData(packet);
                intake.sendPlotData(packet);
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