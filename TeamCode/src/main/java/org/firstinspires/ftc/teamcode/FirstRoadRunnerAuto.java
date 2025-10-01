package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="First RoadRunner Auto")
public class FirstRoadRunnerAuto extends LinearOpMode {
    public static double DISTANCE_IN = 64;

    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(0));
        TankDrive drive = new TankDrive(hardwareMap, initialPose);
        Servo s = hardwareMap.servo.get("servo0");

        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .lineToX(DISTANCE_IN)
                        .stopAndAdd(new MoveServoPatient(s, 0, 3.0))
                        .stopAndAdd(new MoveServo(s, 0.5))
                        .lineToX(0)
                        .stopAndAdd(new MoveServoPatient(s, 1, 3.0))
                        .build()
        );
    }

    public class MoveServo implements Action {
        Servo servo;
        double targetPosition;

        public MoveServo(Servo servo, double targetPosition) {
            this.servo = servo;
            this.targetPosition = targetPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(this.targetPosition);
            return false;
        }
    }

    public class MoveServoPatient implements Action {
        Servo servo;
        double targetPosition;
        double waitTime_s;
        ElapsedTime timer;
        public MoveServoPatient(Servo servo, double targetPosition, double waitTime_s) {
            this.servo = servo;
            this.targetPosition = targetPosition;
            this.waitTime_s = waitTime_s;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer==null) {
                timer = new ElapsedTime();
            }

            servo.setPosition(this.targetPosition);
            return timer.seconds() < waitTime_s;
        }

    }
}
