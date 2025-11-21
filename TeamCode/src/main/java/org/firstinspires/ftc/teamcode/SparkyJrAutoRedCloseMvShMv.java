package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Red-Jr-Close-Mv-Sh-Mv", group="Robot", preselectTeleOp = "JrTeleOp")
@Config
@Disabled
public class SparkyJrAutoRedCloseMvShMv extends SparkyJrCommonLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SparkyJrAutonActionsFactory actionsFactory = new SparkyJrAutonActionsFactory(true);

        intitializeAuton(actionsFactory.initialPose);

        Action runningAction = actionsFactory.buildAction(
                drive, shooter, SparkyJrAutonActionsFactory.StopPose.STOP_AT_POSE3);
        runningActionsList.add(runningAction);

        loopBody();
    }
}
