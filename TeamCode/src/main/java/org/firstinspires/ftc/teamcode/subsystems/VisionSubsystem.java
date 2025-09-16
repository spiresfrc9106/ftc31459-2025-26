package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    //TODO: Make vision do something vision-y
    public VisionSubsystem() {

    }

    public int checkObeliskState() {
        //if (cantSeeObelisk) return -1;
        //else return obeliskPosition12or3
        return 1;
    }

    public double getDistanceToTarget(boolean onBlue) {
        //if (onBlue) return distanceToBlueGoal;
        //else return distanceToRedGoal
        return 1;
    }
}
