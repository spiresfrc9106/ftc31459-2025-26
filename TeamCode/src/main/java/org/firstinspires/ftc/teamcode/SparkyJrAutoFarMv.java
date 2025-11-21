package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Jr-Far-Mv", group="Robot", preselectTeleOp = "JrTeleOp")
@Config
@Disabled
public class SparkyJrAutoFarMv extends SparkyJrCommonLinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SparkyJrAutonActionsFactory actionsFactory = new SparkyJrAutonActionsFactory(false);

        intitializeAuton(actionsFactory.initialPose);

        Action runningAction = actionsFactory.buildActionDriveOut(drive, shooter);
        runningActionsList.add(runningAction);

        loopBody();
    }
}
