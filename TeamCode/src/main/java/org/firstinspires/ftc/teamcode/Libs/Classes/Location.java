package org.firstinspires.ftc.teamcode.Libs.Classes;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Libs.Classes.Vector3;

public class Location implements Runnable{
    //Motors
    public DcMotor Left;
    public DcMotor Right;
    public DcMotor Center;

    //Math
    public double lateralDistance;
    public double DistanceFromCenter;
    public double RadiusOfWheels;
    public double NumberOfTicks;
    public double CMPerTick;
    public Vector3 Position;
    public double AngleRaw;

    //Old values
    double Old_left;
    double Old_right;
    double Old_center;

    //new values
    double New_left;
    double New_right;
    double New_center;

    //Calc
    double dnL;
    double dnR;
    double dnB;
    double Turn;
    double X;
    double Y;

    //Optimization vars
    double Dx;
    double Dy;
    double Dt;

    //Telemetry
    Telemetry telemetry;

    //Private Functions



    //Public Functions

    public Location(DcMotor Left, DcMotor Right, DcMotor Center, double lateralDistance, double DistanceFromCenter, double RadiusOfWheels, double NumberOfTicks, Telemetry telemetry){
        this.Left = Left;
        this.Right = Right;
        this.Center = Center;
        this.lateralDistance = lateralDistance;
        this.DistanceFromCenter = DistanceFromCenter;
        this.RadiusOfWheels = RadiusOfWheels;
        this.NumberOfTicks = NumberOfTicks;
        this.telemetry = telemetry;

        this.AngleRaw = 6749;
        this.CMPerTick = (2 * Math.PI * RadiusOfWheels) / NumberOfTicks;

        //Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Initialize position
        Position = new Vector3(0,0,0);
    }

    public void calculatePosition(){

        New_left = -Left.getCurrentPosition();
        New_right = -Right.getCurrentPosition();
        New_center = Center.getCurrentPosition();

        int dn1 = (int) (New_left - Old_left);
        int dn2 = (int) (New_right - Old_right);
        int dn3 = (int) (New_center - Old_center);

        //Calculate the change in angle
        double dtheta = CMPerTick * (dn1 - dn2) / lateralDistance;
        double dx = CMPerTick * (dn1 + dn2) / 2;
        double dy = CMPerTick * (dn3 - (dn1 - dn2) * DistanceFromCenter / lateralDistance);

        //Update the position
        double theta = Position.z + (dtheta / 2);
        Position.x += dx * Math.cos(theta) - dy * Math.sin(theta);
        Position.y += dx * Math.sin(theta) + dy * Math.cos(theta);
        Position.z += dtheta;

        Old_left = Left.getCurrentPosition();
        Old_right = Right.getCurrentPosition();
        Old_center = Center.getCurrentPosition();

    }

    public void run(){

        New_left = Left.getCurrentPosition();
        New_right = Right.getCurrentPosition();
        New_center = Center.getCurrentPosition();

        dnL = (New_left - Old_left);
        dnR = -(New_right - Old_right);
        dnB = (New_center - Old_center);

        dnL *= CMPerTick;
        dnR *= CMPerTick;
        dnB *= CMPerTick;

        //invert R
        //dnR *= -1;

        Turn = dnL - dnR;
        X = ((dnL + -dnR) / 2);
        Y = dnB + (Turn*2);

        Dx += X * Math.cos(getRadians()) + Y * Math.sin(getRadians());
        Dy += X * Math.sin(getRadians()) + Y * Math.cos(getRadians());
        Dt += Turn;

        Old_left = Left.getCurrentPosition();
        Old_right = Right.getCurrentPosition();
        Old_center = Center.getCurrentPosition();

    }

    public Vector3 getPosition() {

        //Update the position
        Position.x = Dx;
        Position.y = Dy;
        Position.z = Dt;

        //print
        ///telemetry.addData("dnL", dnL);
        ///telemetry.addData("dnR", dnR);
        ///telemetry.addData("dnB", dnB);

        ///telemetry.addData("Old_left", Old_left);
        ///telemetry.addData("Old_right", Old_right);
        ///telemetry.addData("Old_center", Old_center);


        return Position;
    }

    public double getAngle(){
        return (Dt / 6749) * 360;
    }
    public double getRadians(){
        return getAngle() * (Math.PI / 180);
    }

   /* public void reset(){
        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Center.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Center.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }*/

}
