package org.firstinspires.ftc.teamcode;

// import static org.firstinspires.ftc.teamcode.RobotConstants.state;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TankDrive;



import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

//@Autonomous(name="Auto Blue BB Nando Mocanu numaru 1", group = "advanced")
@Autonomous(name="Robot: AutoV2", group="Robot")
@Config
public class AutoV2 extends LinearOpMode {

    public static double startNow = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        int level=0;

        Pose2d initialPose = new Pose2d(new Vector2d(-24.0,0), Math.toRadians(0));
        Pose2d secondPose = new Pose2d(new Vector2d(-24.0+1.0,0), Math.toRadians(0));

        TelemetryPacket packet = new TelemetryPacket();

        FtcDashboard dash = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        TankDrive drive = new TankDrive(hardwareMap, initialPose);
        ElapsedTime timer1 = new ElapsedTime();

        if (isStopRequested()) return;

        VelConstraint endVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(10),
                        new AngularVelConstraint(5)
                ));

        AccelConstraint endAccelConstraint = new ProfileAccelConstraint(-5, 10);

        Action dummyStart = new SequentialAction(new SleepAction(3.0));

        Action a = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-8, -10), Math.toRadians(60))
                .splineTo(new Vector2d(0, 15), Math.toRadians(90), endVelConstraint, endAccelConstraint )
                .splineTo(new Vector2d(0, 24), Math.toRadians(90), endVelConstraint, endAccelConstraint )
                .build();

        List<Action> dummyActions = new ArrayList<>();
        dummyActions.add(dummyStart);

        List<Action> runningActions = new ArrayList<>();

        runningActions.add(a);

        List<Action> doActions = dummyActions;

        boolean start = false;

        // Wait for the game to start (driver presses START)
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (!start && startNow>0.0) {
                start = true;
                doActions = runningActions;
            }
            else if (!start && doActions.isEmpty()) {
                doActions.add(dummyStart);
            }

            if (!start) {
                drive.sendPlotData(packet);
            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : doActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            doActions = newActions;

            telemetry.addData("Runtime", getRuntime());

            telemetry.update(); // send data from the telemetry.addData calls


        }
    }
}