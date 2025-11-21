package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red-Sr-Close-Mv-Sh-Mv", group="Robot", preselectTeleOp = "SrTeleOp")
@Config
public class SparkySrAutoRedCloseMvShMv extends SparkySrCommonLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SparkySrAutonActionsFactory actionsFactory = new SparkySrAutonActionsFactory(true,SparkySrAutonActionsFactory.StartPose.START_AT_GOAL);

        intitializeAuton(actionsFactory.initialPose);

        Action runningAction = actionsFactory.buildAction(
                drive, shooter, SparkySrAutonActionsFactory.StopPose.STOP_AT_POSE3);
        runningActionsList.add(runningAction);

        loopBody();
    }
}
