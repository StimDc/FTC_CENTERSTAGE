package org.firstinspires.ftc.teamcode.Implementations.Math;

import static org.firstinspires.ftc.teamcode.Implementations.Constants.UniversalConsts.GO_TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.UniversalConsts.WHEEL_CIRCUMFERENCE;

public class MathFunc {

    public static double inchToTicksD(double inch){
        return ((inch/WHEEL_CIRCUMFERENCE)*GO_TICKS_PER_REV);
    }
    public static int inchToTicks(double inch){
        return (int)((inch/WHEEL_CIRCUMFERENCE)*GO_TICKS_PER_REV);
    }

    public static double cmToInch(double cm){
        return cm * 0.393700787;
    }

    public static double ticksToInch(double ticks){
        return (ticks/GO_TICKS_PER_REV)*WHEEL_CIRCUMFERENCE;
    }

    public static double MaxPower(double a, double b, double c, double d){

        return Math.max(a,Math.max(b,Math.max(c,d)));



    }


}