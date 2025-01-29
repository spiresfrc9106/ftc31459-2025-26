package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Juyoung9thMVP", group="robotgroup")
public class teleOp extends OpMode {

    boolean inital = true;

    private hardwareHandler HardwareHandler;
    @Override
    public void init() {
        HardwareHandler = new hardwareHandler(hardwareMap, telemetry);

    }
    @Override
    if (inital) {
        HardwareHandler.intakeSystem(1);
    }



        if (aCurrState && !aPrevState){
        if (!intakeIn){

        }
    }
}
