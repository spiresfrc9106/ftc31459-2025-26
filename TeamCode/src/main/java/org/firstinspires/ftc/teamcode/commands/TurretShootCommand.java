package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretShootCommand extends CommandBase {
    TurretSubsystem turretSubsystem;
    double target;

    public TurretShootCommand(TurretSubsystem turretSubsystem, double target) {
        this.turretSubsystem = turretSubsystem;
        this.target = target;
    }

    @Override
    public void initialize() {
        if(turretSubsystem.AimTurret(target)) turretSubsystem.FireTurret();
        else {} //TODO: Do something
    }
}
