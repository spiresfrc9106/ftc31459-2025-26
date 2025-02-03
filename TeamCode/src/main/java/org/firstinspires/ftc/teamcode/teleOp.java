package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

package org.firstinspires.ftc.teamcode;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Juyoung9thMVP", group="robotgroup")
public class teleOp extends OpMode {

    boolean aG2PrevState = false;
    boolean aG2CurrState = false;
    boolean bG2PrevState = false;
    boolean bG2CurrState = false;
    boolean intakeIn = false;
    boolean intakeOut = false;
    boolean inital = true;
    boolean reeling = false;
    boolean reelUpCurrState = false;
    boolean reelUpPrevState = false;
    boolean stage2ClimbUp = false;
    boolean s2CUpPrevState = false;
    boolean s2CUpCurrState = false;
    boolean s2CDownCurrState = false;
    boolean s2CDownPrevState = false;
    boolean stage2ClimbDown = false;
    boolean reelDownCurrState = false;
    boolean reelDownPrevState = false;
    boolean reelIn = false;
    boolean tMFUpPrevState = false;
    boolean tMFUpCurrState = false;
    boolean tmfUp = false;
    boolean tMFDownCurrState = false;
    boolean tMFDownPrevState = false;
    boolean tmfDown = false;
    double slowMode = 1;
    boolean slowPrev = false;
    boolean slowCurr = false;
    boolean slowOn = false;


    private hardwareHandler HardwareHandler;
    @Override
    public void init() {
        HardwareHandler = new hardwareHandler(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        if (inital){
            HardwareHandler.intakeAngle(0);
            HardwareHandler.aimSystem(0);
            inital = !(inital);
        }

        boolean aG2CurrState = gamepad2.a;
        boolean bG2CurrState = gamepad2.b;
        boolean reelUpCurrState = gamepad2.dpad_up;
        boolean tMFUpCurrState = gamepad2.dpad_up;
        boolean reelDownCurrState = gamepad2.dpad_down;
        boolean tMFDownCurrState = gamepad2.dpad_down;
        boolean s2CUpCurrState = gamepad2.x;
        boolean s2CDownCurrState = gamepad2.y;


            if (slowCurr && !slowPrev){
                if (!slowOn)
                    slowMode = 0.6;
                else
                    slowMode = 1.0;
                slowOn = !(slowOn);
            }



        double c = 0.7;
        double f = gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x * 0.5 / 0.65;
        double s = gamepad1.left_stick_x;
        double speed = Math.max(Math.max(f * f, r * r), s * s) * c * slowMode;
        HardwareHandler.moveWithPower(f, r, s, speed);


            if (aG2CurrState && !aG2PrevState){
                if (!intakeIn)
                HardwareHandler.intakeSystem(-1);
                else
                HardwareHandler.intakeSystem(0);
                (intakeIn) = !(intakeIn);

            if (reelUpCurrState && !(reelUpPrevState)) {
                if (!reeling)
                    HardwareHandler.reeling(1);
                else
                    HardwareHandler.reeling(0);
                reeling = !(reeling);
            }

            if (tMFUpCurrState && !tMFUpPrevState)
                if (!tmfUp)
                    HardwareHandler.tapeMeasureFire(1);
                else
                    HardwareHandler.tapeMeasureFire(0);
                tmfUp = !(tmfUp);


            if (tMFDownCurrState && !tMFDownPrevState){
                if (!tmfDown)
                    HardwareHandler.tapeMeasureFire(-1);
                else
                    HardwareHandler.tapeMeasureFire(0);
                tmfDown = !(tmfDown);
            }


            if (reelDownCurrState && !reelDownPrevState){
                if (reelIn)
                    HardwareHandler.reeling(-1);
                else
                    HardwareHandler.reeling(0);
                reelIn = !(reelIn);

            }

            HardwareHandler.linearLift(gamepad2.left_stick_y*0.5);

            if (bG2CurrState && !bG2PrevState){
                if (!intakeOut)
                HardwareHandler.intakeSystem(1);
                else
                HardwareHandler.intakeSystem(0);
                (intakeOut) = !(intakeOut);
            }

            if (s2CUpCurrState && !s2CUpPrevState){
                if (!stage2ClimbUp)
                    HardwareHandler.s2Climb(1);
                else
                    HardwareHandler.s2Climb(0);
                (stage2ClimbUp) = !(stage2ClimbUp);
            }

            if (s2CDownCurrState && !s2CDownPrevState){
                if (!stage2ClimbDown)
                    HardwareHandler.s2Climb(-1);
                else
                    HardwareHandler.s2Climb(0);
                (stage2ClimbDown) = !(stage2ClimbDown);
            }

                if (gamepad1.y)
                    HardwareHandler.intakeAngle(0);

                if (gamepad1.x)
                    HardwareHandler.intakeAngle(0.2);

                if (gamepad1.a)
                    HardwareHandler.intakeAngle(0.3);

                HardwareHandler.aimSystem(gamepad2.right_stick_y);
        }
        s2CDownPrevState = s2CDownCurrState;
        s2CUpPrevState = s2CUpCurrState;
        aG2PrevState = aG2CurrState;
        bG2PrevState = bG2CurrState;
        reelUpPrevState = reelUpCurrState;
        reelDownPrevState = reelDownCurrState;
        tMFUpPrevState = tMFUpCurrState;
        tMFDownPrevState = tMFDownCurrState;
        slowPrev = slowCurr;

    }

}

