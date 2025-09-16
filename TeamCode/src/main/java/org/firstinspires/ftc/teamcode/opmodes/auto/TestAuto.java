package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.ActionCommand;
import org.firstinspires.ftc.teamcode.commands.CheckObeliskCommand;
import org.firstinspires.ftc.teamcode.commands.TurretShootCommand;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@Config
@Autonomous(name="Test Auto", group="test")
public class TestAuto extends CommandOpMode {
    @Override
    public void initialize() {
        Pose2d initialPose = new Pose2d(61, 0, 0);
        Pose2d obeliskPose = new Pose2d(-61, 0, 0);
        Pose2d artifact1Pose = new Pose2d(-11, 40, 90);
        Pose2d artifact2Pose = new Pose2d(11, 40, 90);
        Pose2d artifact3Pose = new Pose2d(33, 40, 90);
        Pose2d shootPose = new Pose2d(-22, 22, 45);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        VisionSubsystem visionSubsystem = new VisionSubsystem();
        TurretSubsystem turretSubsystem = new TurretSubsystem(hardwareMap);

        Action driveToObelisk = drive.actionBuilder(initialPose)
                .splineToLinearHeading(obeliskPose, Math.toRadians(0))
                .build();

        Action driveToArtifact1 = drive.actionBuilder(obeliskPose)
                .splineToLinearHeading(artifact1Pose, Math.toRadians(90))
                .build();

        Action driveToArtifact2 = drive.actionBuilder(obeliskPose)
                .splineToLinearHeading(artifact2Pose, Math.toRadians(90))
                .build();

        Action driveToArtifact3 = drive.actionBuilder(obeliskPose)
                .splineToLinearHeading(artifact3Pose, Math.toRadians(90))
                .build();

        Action driveToShootFrom1 = drive.actionBuilder(artifact1Pose)
                .splineToLinearHeading(shootPose, Math.toRadians(45))
                .build();
        Action driveToShootFrom2 = drive.actionBuilder(artifact2Pose)
                .splineToLinearHeading(shootPose, Math.toRadians(45))
                .build();
        Action driveToShootFrom3 = drive.actionBuilder(artifact3Pose)
                .splineToLinearHeading(shootPose, Math.toRadians(45))
                .build();

        Action driveTohome = drive.actionBuilder(shootPose)
                .splineToLinearHeading(initialPose, 0)
                .build();

        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(
                        new SequentialCommandGroup(
                                new ActionCommand(driveToObelisk),
                                //TODO: Intake
                                new CheckObeliskCommand(
                                        new SequentialCommandGroup(
                                                new ActionCommand(driveToArtifact1),
                                                new ActionCommand(driveToShootFrom1)
                                        ),
                                        new SequentialCommandGroup(
                                                new ActionCommand(driveToArtifact2),
                                                new ActionCommand(driveToShootFrom2)
                                        ),
                                        new SequentialCommandGroup(
                                                new ActionCommand(driveToArtifact3),
                                                new ActionCommand(driveToShootFrom3)
                                        ),
                                        new InstantCommand(() -> {}),
                                        visionSubsystem
                                ),
                                new TurretShootCommand(turretSubsystem, () -> {return visionSubsystem.getDistanceToTarget(false);}),
                                new ActionCommand(driveTohome)
                        )
                )
        );
    }
}
