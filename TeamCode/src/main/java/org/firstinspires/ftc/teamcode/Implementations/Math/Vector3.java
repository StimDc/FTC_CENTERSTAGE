package org.firstinspires.ftc.teamcode.Implementations.Math;

import org.firstinspires.ftc.teamcode.Implementations.Annotations.ImplementedBy;


public class Vector3 {
    public double x, y,z;
    public Vector3(double x, double y, double z){
        setVector(x,y,z);
    }
    public Vector3(){

    }
    public void setVector(double x,double y,double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

}
