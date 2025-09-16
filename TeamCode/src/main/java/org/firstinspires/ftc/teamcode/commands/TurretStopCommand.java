package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretStopCommand extends CommandBase {
    TurretSubsystem turretSubsystem;
    public TurretStopCommand(TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
    }

    @Override
    public void execute() {
        turretSubsystem.stopTurret();
    }

    @Override
    public boolean isFinished() {
        return !turretSubsystem.isTurretMoving();
    }
}
