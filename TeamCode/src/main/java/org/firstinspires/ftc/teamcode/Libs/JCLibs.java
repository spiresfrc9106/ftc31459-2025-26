package org.firstinspires.ftc.teamcode.Libs;

public class JCLibs {
    public static double lerp(double a, double b, double t){
        return a + (b - a) * t;
    }
    //A is the start, B is the end, T is the percentage

    public static double round(double num, int places){
        double mult = Math.pow(10, places);
        return Math.round(num * mult) / mult;
    }

    public static double clamp(double num, double min, double max){
        if(num < min){
            return min;
        }else if(num > max){
            return max;
        }else{
            return num;
        }
    }

    public static double clamp(double num, double max){
        return clamp(num, -max, max);
    }

    public static double clamp(double num){
        return clamp(num, 1);
    }

    public static String GetTime(){
        return java.text.DateFormat.getTimeInstance().format(new java.util.Date());
    }

    public static String GetDate(){
        return java.text.DateFormat.getDateInstance().format(new java.util.Date());
    }

    public static double Distance1D(double a, double b){
        return Math.abs(a - b);
    }

    public static double Distance2D(double x1, double y1, double x2, double y2){
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    public static double Map(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static boolean IsDistanceClose(double a, double b, double threshold){
        return Distance1D(a, b) < threshold;
    }
}
