package org.firstinspires.ftc.teamcode.Libs.Classes;

public class Vector3 {
    public double x;
    public double y;
    public double z;

    public Vector3(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3(double x, double y){
        this(x, y, 0);
    }

    public Vector3(){
        this(0,0,0);
    }

    public Vector3 Lerp(Vector3 start, Vector3 end, double t){
        return new Vector3(
            start.x + (end.x - start.x) * t,
            start.y + (end.y - start.y) * t,
            start.z + (end.z - start.z) * t
        );
    }

    public Vector3 Lerp(Vector3 end, double t){
        return Lerp(this, end, t);
    }

    public Vector3 Add(Vector3 other){
        return new Vector3(
            x + other.x,
            y + other.y,
            z + other.z
        );
    }

    public Vector3 Add(double x, double y, double z){
        return Add(new Vector3(x, y, z));
    }

    public Vector3 Add(double x, double y){
        return Add(new Vector3(x, y));
    }


    public Vector3 Sub(Vector3 other){
        return new Vector3(
            x - other.x,
            y - other.y,
            z - other.z
        );
    }

    public Vector3 Sub(double x, double y, double z){
        return Sub(new Vector3(x, y, z));
    }

    public Vector3 Sub(double x, double y){
        return Sub(new Vector3(x, y));
    }

    public double Dot(Vector3 other){
        return x * other.x + y * other.y + z * other.z;
    }

    public double Dot(double x, double y, double z){
        return Dot(new Vector3(x, y, z));
    }

    public Vector3 Cross(Vector3 other){
        return new Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    public Vector3 Cross(double x, double y, double z){
        return Cross(new Vector3(x, y, z));
    }

    public double Magnitude(){
        return Math.sqrt(x * x + y * y + z * z);
    }

    public Vector3 Normalize(){
        double mag = Magnitude();
        return new Vector3(x / mag, y / mag, z / mag);
    }

    public Vector3 Scale(double scalar){
        return new Vector3(x * scalar, y * scalar, z * scalar);
    }

    public double Distance(Vector3 other){
        return Math.sqrt(Math.pow(x - other.x, 2) + Math.pow(y - other.y, 2) + Math.pow(z - other.z, 2));
    }

    public double Distance(double x, double y, double z){
        return Distance(new Vector3(x, y, z));
    }

    public double Average(){
        return (x + y + z) / 3;
    }


    public double[] ToArray(){
        return new double[]{x, y, z};
    }
}
