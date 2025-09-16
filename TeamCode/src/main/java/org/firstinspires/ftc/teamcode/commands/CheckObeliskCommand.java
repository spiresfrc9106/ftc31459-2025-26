package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class CheckObeliskCommand extends CommandBase {
    Command ifPosition0;
    Command ifPosition1;
    Command ifPosition2;
    Command ifNone;
    VisionSubsystem visionSubsystem;
    Command selectedCommand;
    public CheckObeliskCommand(Command ifPosition0, Command ifPosition1, Command ifPosition2, Command ifNone, VisionSubsystem visionSubsystem) {
        this.ifPosition0 = ifPosition0;
        this.ifPosition1 = ifPosition1;
        this.ifPosition2 = ifPosition2;
        this.ifNone = ifNone;
        this.visionSubsystem = visionSubsystem;
    }

    @Override
    public void initialize() {
        switch (visionSubsystem.checkObeliskState()) {
            case 0:
                selectedCommand = ifPosition0;
                break;
            case 1:
                selectedCommand = ifPosition1;
                break;
            case 2:
                selectedCommand = ifPosition2;
                break;
            default:
                selectedCommand = ifNone;
        }
    }

    @Override
    public void execute() {
        selectedCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return selectedCommand.isFinished();
    }
}
