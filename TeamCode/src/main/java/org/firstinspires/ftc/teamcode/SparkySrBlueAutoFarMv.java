package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue-Sr-Far-Sh-SparkySrBlueAutoFarMvMv", group="Robot", preselectTeleOp = "SrTeleOp")
@Config
public class SparkySrBlueAutoFarMv extends SparkySrCommonLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SparkySrAutonActionsFactory actionsFactory = new SparkySrAutonActionsFactory(false, SparkySrAutonActionsFactory.StartPose.START_AT_SMALL_TRIANGLE);

        intitializeAuton(actionsFactory.initialPose);

        Action runningAction = actionsFactory.buildActionFarShootDriveOut(
                drive, shooter);
        runningActionsList.add(runningAction);

        loopBody();
    }
}
