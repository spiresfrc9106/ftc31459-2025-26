package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.ArrayList;

@Autonomous(name="Red-Jr-Close-Mv-Sh", group="Robot", preselectTeleOp = "JrTeleOp")
@Config
public class SparkyJrAutoRedCloseMvSh extends SparkyJrCommonLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SparkyJrAutonActionsFactory actionsFactory = new SparkyJrAutonActionsFactory(true);

        intitializeAuton(actionsFactory.initialPose);

        Action runningAction = actionsFactory.buildAction(drive, shooter, SparkyJrAutonActionsFactory.StopPose.STOP_AT_POSE2);
        runningActionsList.add(runningAction);

        loopBody();
    }
}
