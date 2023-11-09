package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.Implementations.Annotations.ImplementedBy;


public class Vector3 {
    public double x, y,z;
    Vector3(double x, double y, double z){
        setVector(x,y,z);
    }
    Vector3(){

    }
    void setVector(double x,double y,double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

}
