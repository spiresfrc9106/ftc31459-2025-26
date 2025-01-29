package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.time.chrono.IsoEra;

public class hardwareHandler {

    private final DcMotor wheelFR;
    private final DcMotor wheelFL;
    private final DcMotor wheelRR;
    private final DcMotor wheelRL;
    private final DcMotor linearL;
    private final DcMotor linearR;
    private final DcMotor raiseThingy;
    private final DcMotor reel;

    private final CRServo intakeR;
    private final CRServo intakeL;
    private final CRServo rTMF;
    private final CRServo lTMF;

    private final Servo raiseR;
    private final Servo raiseL;
    private final Servo lTMA;
    private final Servo rTMA;

    private final int ESFR = 1;
    private final int ESFL = 1;
    private final int ESRR = 1;
    private final int ESRL = 1;

    private Telemetry telemetry;


public hardwareHandler (HardwareMap thomasHardwareMap, Telemetry telemetry) {

wheelFL = thomasHardwareMap.dcMotor.get("LeftFront");
wheelFR = thomasHardwareMap.dcMotor.get("RightFront");
wheelRL = thomasHardwareMap.dcMotor.get("LeftRear");
wheelRR = thomasHardwareMap.dcMotor.get("RightRear");
linearL = thomasHardwareMap.dcMotor.get("LinearL");
linearR = thomasHardwareMap.dcMotor.get("LinearR");
intakeR = thomasHardwareMap.crservo.get("IntakeR");
intakeL = thomasHardwareMap.crservo.get("IntakeL");
lTMA = thomasHardwareMap.servo.get("LeftTapeMeasureAim");
rTMA = thomasHardwareMap.servo.get("RightTapeMeasureAim");
raiseThingy = thomasHardwareMap.dcMotor.get("raisethingy");
reel = thomasHardwareMap.dcMotor.get("Reel");
raiseL = thomasHardwareMap.servo.get("raiseL");
raiseR = thomasHardwareMap.servo.get("raiseR");
rTMF = thomasHardwareMap.crservo.get("RightTapeMeasureFire");
lTMF = thomasHardwareMap.crservo.get("LeftTapeMeasureFire");

wheelFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
wheelRL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
wheelFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
wheelRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
linearL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
linearR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
raiseThingy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
reel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

wheelFL.setDirection(DcMotorSimple.Direction.REVERSE);
wheelRL.setDirection(DcMotorSimple.Direction.REVERSE);

wheelRL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
wheelRR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
wheelFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
wheelFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
linearR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
linearL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
raiseThingy.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
reel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

wheelRL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
wheelRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
wheelFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
wheelFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
linearR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
linearL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
raiseThingy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
reel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

}
    public void moveWithPower(double d, double r, double s, double speed) { // d : linear movement, r : rotational movement, s : speed (0-1); r is signed with CCW as positive
        //assert (speed <= 1 && speed >= 0): "Speed must be between 0 and 1";

        double total = Math.abs(d) + Math.abs(r) + Math.abs(s);
        if (d == 0 && r == 0 && s == 0) {
            wheelFL.setPower(0);
            wheelRL.setPower(0);

            wheelFR.setPower(0);
            wheelRR.setPower(0);
        }
        else {
            wheelFL.setPower((-d + r + s) / total * speed * ESFL);
            wheelRL.setPower((-d + r - s) / total * speed * ESRL); // test to change these values
            wheelFR.setPower((-d - r - s) / total * speed * ESFR);
            wheelRR.setPower((-d - r + s) / total * speed * ESRR);

            /*
            d + r + s
            d + r - s
            d - r - s
            d - r + s
             */
        }

    }

    public void linearLift(double power){
        linearL.setPower(-power);
        linearR.setPower(power);
    }
    public void intakeAngle(double pos){
        raiseL.setPosition(0.01-pos);
        raiseR.setPosition(1+pos);
    }
    public void intakeSystem(double power) {
        intakeL.setPower(-power);
        intakeR.setPower(power);
    }
    public void aimSystem(double direction) {
        double leftCurrPos = lTMA.getPosition();
        double rightCurrPos = rTMA.getPosition();
        if ((leftCurrPos > 0.7 && direction < 0) || (rightCurrPos > 0 && direction > 0)) {
            lTMA.setPosition(leftCurrPos + direction * 0.001);
            rTMA.setPosition(rightCurrPos + direction * -0.001);
        }
    }
    public void reeling(double power){
        reel.setPower(power);
    }
    public void tapeMeasureFire(double power){
        lTMF.setPower(-power);
        rTMF.setPower(power);
    }
    public void s2Climb(double power){
        raiseThingy.setPower(power);
    }

}

