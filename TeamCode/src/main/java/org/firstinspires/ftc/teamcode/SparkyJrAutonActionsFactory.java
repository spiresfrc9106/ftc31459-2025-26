package org.firstinspires.ftc.teamcode;

// import static org.firstinspires.ftc.teamcode.RobotConstants.state;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class SparkyJrAutonActionsFactory {

    public static double autonMoveFactor = 0.5;
    public static double autonRotationFactor = 0.5;
    public static double autonMoveAccelFactor = 0.5;

    public List<Action> runningActions = new ArrayList<>();

    double PlusOrMinusOne = 1.0;

    TankDrive drive = null;
    SparkyJrShooter shooter = null;

    public Pose2d initialPose = null;

    public enum StopPose {
        STOP_AT_POSE2,
        STOP_AT_POSE3,
        STOP_AT_POSE4,
    }

    SparkyJrAutonActionsFactory(boolean isRed){
        if (isRed) {
            PlusOrMinusOne = -1.0;
        }
        initialPose = new Pose2d(new Vector2d(53, 51*PlusOrMinusOne), Math.toRadians(-114*PlusOrMinusOne));
    }

    public Action buildAction(TankDrive drive, SparkyJrShooter shooter, StopPose stopPose) {
        this.drive = drive;
        this.shooter = shooter;
        Pose2d secondPose = new Pose2d(new Vector2d(41, 39*PlusOrMinusOne), Math.toRadians(-120*PlusOrMinusOne));
        Pose2d thirdPose = new Pose2d(new Vector2d( 19, 24*PlusOrMinusOne), Math.toRadians(-179*PlusOrMinusOne));
        Pose2d fourthPose = new Pose2d(new Vector2d( -30, -53*PlusOrMinusOne), Math.toRadians(-135*PlusOrMinusOne));

        VelConstraint velConstraint =
                new MinVelConstraint(Arrays.asList(
                        drive.kinematics.new WheelVelConstraint(drive.PARAMS.maxWheelVel* autonMoveFactor),
                        new AngularVelConstraint(drive.PARAMS.maxAngVel* autonRotationFactor)
                ));

        AccelConstraint accelConstraint = new ProfileAccelConstraint(drive.PARAMS.minProfileAccel* autonMoveAccelFactor, drive.PARAMS.maxProfileAccel* autonMoveAccelFactor);

        Action actionDrive = drive.actionBuilder(initialPose)
                .splineTo(secondPose.position,secondPose.heading, velConstraint, accelConstraint)
                .build();

        List<Action> listOfActions = new ArrayList<>();
        listOfActions.add(new ParallelAction(actionDrive, shooter.new SpinUpAutonomous()));
        listOfActions.add(new SleepAction(1.0));
        listOfActions.add(shooter.new LaunchAutonomous());
        listOfActions.add(new SleepAction(2.0));
        listOfActions.add(shooter.new LaunchAutonomous());
        listOfActions.add(new SleepAction(2.0));
        listOfActions.add(shooter.new LaunchAutonomous());
        listOfActions.add(new SleepAction(1.0));
        listOfActions.add(shooter.new SpinDownAutonomous());
        switch (stopPose) {
            case STOP_AT_POSE2:
                break;
            case STOP_AT_POSE3:
                listOfActions.add(drive.actionBuilder(secondPose)
                        .splineTo(thirdPose.position, thirdPose.heading, velConstraint, accelConstraint)
                        .build());
                break;
            case STOP_AT_POSE4:
                listOfActions.add(drive.actionBuilder(secondPose)
                        .splineTo(thirdPose.position, thirdPose.heading, velConstraint, accelConstraint)
                        .splineTo(fourthPose.position, fourthPose.heading, velConstraint, accelConstraint)
                        .build());
                break;
        }

        Action actionWhileFirstDrive = new SequentialAction(listOfActions);

        return actionWhileFirstDrive;

    }

}
