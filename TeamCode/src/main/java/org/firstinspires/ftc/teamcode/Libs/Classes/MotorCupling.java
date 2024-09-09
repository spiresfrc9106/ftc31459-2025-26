package org.firstinspires.ftc.teamcode.Libs.Classes;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorCupling {
    public DcMotor DC1;
    public DcMotor DC2;

    private int offset1 = 0;
    private int offset2 = 0;


    public void SetOffset(int offset, int DC){
        if(DC == 1){
            offset1 = offset;
        }else if(DC == 2){
            offset2 = offset;
        }else {
            //Error
            throw new IllegalArgumentException("DC must be 1 or 2");
        }

    }


    public void setPower(double power){
        DC1.setPower(power);
        DC2.setPower(power);
    }

    public void setDirection(DcMotor.Direction direction){
        DC1.setDirection(direction);
        DC2.setDirection(direction);
    }

    public void setMode(DcMotor.RunMode runMode){
        DC1.setMode(runMode);
        DC2.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior){
        DC1.setZeroPowerBehavior(zeroPowerBehavior);
        DC2.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setTargetPosition(int position){
        DC1.setTargetPosition(position + offset1);
        DC2.setTargetPosition(position + offset2);
    }

    public int getCurrentPosition(){
        return DC1.getCurrentPosition();
    }

    public void setPowerFloat(){
        DC1.setPowerFloat();
        DC2.setPowerFloat();
    }

    public void setTargetPosition(int position1, int position2){
        DC1.setTargetPosition(position1 + offset1);
        DC2.setTargetPosition(position2 + offset2);
    }

    public void setMode(DcMotor.RunMode runMode1, DcMotor.RunMode runMode2){
        DC1.setMode(runMode1);
        DC2.setMode(runMode2);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior1, DcMotor.ZeroPowerBehavior zeroPowerBehavior2){
        DC1.setZeroPowerBehavior(zeroPowerBehavior1);
        DC2.setZeroPowerBehavior(zeroPowerBehavior2);
    }

    public void setDirection(DcMotor.Direction direction1, DcMotor.Direction direction2){
        DC1.setDirection(direction1);
        DC2.setDirection(direction2);
    }

    public void setPower(double power1, double power2){
        DC1.setPower(power1);
        DC2.setPower(power2);
    }

    public void setPowerFloat(DcMotor.ZeroPowerBehavior zeroPowerBehavior1, DcMotor.ZeroPowerBehavior zeroPowerBehavior2){
        DC1.setPowerFloat();
        DC2.setPowerFloat();
    }


    public boolean isBusy(){
        return DC1.isBusy() || DC2.isBusy();
    }


}
