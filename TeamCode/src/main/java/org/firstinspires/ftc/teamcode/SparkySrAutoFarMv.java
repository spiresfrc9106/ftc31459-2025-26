package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Sr-Far-Mv", group="Robot", preselectTeleOp = "SrTeleOp")
@Config
public class SparkySrAutoFarMv extends SparkySrCommonLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SparkySrAutonActionsFactory actionsFactory = new SparkySrAutonActionsFactory(false, SparkySrAutonActionsFactory.StartPose.START_AT_SMALL_TRIANGLE);

        intitializeAuton(actionsFactory.initialPose);

        Action runningAction = actionsFactory.buildActionDriveOut(drive, shooter);
        runningActionsList.add(runningAction);

        loopBody();
    }
}
