package org.firstinspires.ftc.teamcode.Implementations.Constants;

public class UniversalConsts {


    public static final double GO_TICKS_PER_REV= 537.7d;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * 3.7795275d;
    public static final double LATERAL_ERROR = 1.1904761d;
    public static final double ROTATE_ERROR = 0.85672725d;

    public static double MAX_RANGE=0.85,MAX_HEADING=0.8,MAX_YAW=0.8;
    
    // pentru left si center MAX_RANGE=0.185,MAX_HEADING=0.75,MAX_YAW=0.75
    //pentru right MAX_RANGE=0.16,MAX_HEADING=0.5,MAX_YAW=0.5
    public static double RANGE_ERROR_GAIN=0.02, HEADING_ERROR_GAIN=0.01,YAW_ERROR_GAIN=0.015;
    public static final double TICKS_IN_DEGREES=288/(360.0*0.36);
    public static final int SLEEP = 500;
}
