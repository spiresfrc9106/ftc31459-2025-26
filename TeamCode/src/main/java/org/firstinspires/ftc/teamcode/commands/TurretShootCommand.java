package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class TurretShootCommand extends CommandBase {
    TurretSubsystem turretSubsystem;
    DoubleSupplier target;

    public TurretShootCommand(TurretSubsystem turretSubsystem, DoubleSupplier target) {
        this.turretSubsystem = turretSubsystem;
        this.target = target;
    }

    @Override
    public void execute() {
        if(turretSubsystem.aimTurret(target.getAsDouble())) turretSubsystem.fireTurret();
        else {} //TODO: Do something
    }

    @Override
    public boolean isFinished() {
        return turretSubsystem.isTurretMoving();
    }
}
