package org.firstinspires.ftc.teamcode;

// import static org.firstinspires.ftc.teamcode.RobotConstants.state;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
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
@Autonomous(name="Robot: Auto1", group="Robot")
public class AutoV2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        int level=0;

        Pose2d initialPose = new Pose2d(new Vector2d(-20.0,0), Math.toRadians(0));
        TankDrive drive = new TankDrive(hardwareMap, initialPose);
        ElapsedTime timer1 = new ElapsedTime();

        telemetry.addData("nu vad camera",0);
        telemetry.update();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("pos_x", initialPose.component1());
        dashboard.sendTelemetryPacket(packet);
        packet.put("pos_y", initialPose.component2());
        dashboard.sendTelemetryPacket(packet);
        packet.put("pos_h", Math.toDegrees(initialPose.heading.toDouble()));

        dashboard.sendTelemetryPacket(packet);


        while(!isStopRequested() && !isStarted()){
            telemetry.addData("caz: ", 545);
            telemetry.update();
        }

        if (isStopRequested()) return;

        VelConstraint endVelConstraint =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(10),
                        new AngularVelConstraint(5)
                ));

        AccelConstraint endAccelConstraint = new ProfileAccelConstraint(-5, 10);


        Action a = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-8, -10), Math.toRadians(60))
                //.splineTo(new Vector2d(-4, 5), Math.toRadians(75), endVelConstraint, endAccelConstraint )
                //.splineTo(new Vector2d(0, 10), Math.toRadians(90), endVelConstraint, endAccelConstraint )
                .splineTo(new Vector2d(0, 15), Math.toRadians(90), endVelConstraint, endAccelConstraint )
                .splineTo(new Vector2d(0, 20), Math.toRadians(90), endVelConstraint, endAccelConstraint )
                .build();


        /*
        double SIDE_IN = 12.0;
        Action a = drive.actionBuilder(initialPose)
                .forward(SIDE_IN)
                .turn(Math.toRadians(90))
                .forward(SIDE_IN)
                .turn(Math.toRadians(90))
                .forward(SIDE_IN)
                .turn(Math.toRadians(90))
                .forward(SIDE_IN)
                .turn(Math.toRadians(90))
                .build();
         */

        List<Action> runningActions = new ArrayList<>();

        runningActions.add(a);

        while (opModeIsActive() && !isStopRequested()) {
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
                packet.put("pos_x", drive.localizer.getPose().position.x);
                dashboard.sendTelemetryPacket(packet);
                packet.put("pos_y", drive.localizer.getPose().position.y);
                dashboard.sendTelemetryPacket(packet);
                packet.put("pos_h", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));

                dashboard.sendTelemetryPacket(packet);
            }
            runningActions = newActions;

            dashboard.sendTelemetryPacket(packet);

        }
    }
}