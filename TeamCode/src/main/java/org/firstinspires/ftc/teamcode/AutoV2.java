package org.firstinspires.ftc.teamcode;

// import static org.firstinspires.ftc.teamcode.RobotConstants.state;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TankDrive;
/*
import org.firstinspires.ftc.teamcode.RobotConstants.LiftLevels;
import org.firstinspires.ftc.teamcode.systems.Brat;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Lift;
import org.firstinspires.ftc.teamcode.systems.Vision;
 */


import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

//@Autonomous(name="Auto Blue BB Nando Mocanu numaru 1", group = "advanced")
@Autonomous(name="Robot: Auto1", group="Robot")
public class AutoV2 extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {

        int level=0;

        Pose2d initialPose = new Pose2d(new Vector2d(-36.0,0), Math.toRadians(0));
        TankDrive drive = new TankDrive(hardwareMap, initialPose);
        ElapsedTime timer1 = new ElapsedTime();

        telemetry.addData("nu vad camera",0);
        telemetry.update();


        while(!isStopRequested() && !isStarted()){
            telemetry.addData("caz: ", 545);
            telemetry.update();
        }

        if (isStopRequested()) return;


        if (opModeIsActive() && !isStopRequested()) {
            Actions.runBlocking(
                    drive.actionBuilder(initialPose)
                            .splineTo(new Vector2d(0, 20), Math.toRadians(90))
                            .build());

        }
        while (opModeIsActive() && !isStopRequested()) {

        }
    }
}