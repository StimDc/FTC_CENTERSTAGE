package org.firstinspires.ftc.teamcode.Implementations.DebugTools;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class DcMotorDebug{

    public Class<?> motorClass;
    public String ID;
    public DcMotorSimple.Direction direction;
    public DcMotor.RunMode runMode;
    public DcMotor.ZeroPowerBehavior zeroPowerBehavior;
    public float power;
    public int curentPosition;
    public int targetPosition;
    public boolean runToPosition;
    public boolean runUsingEncoders;
    public boolean running;

    public int x,y;
    public float degreeDirection;

    public  DcMotorDebug hardwareMap(Class<?> objectClass,String ID){
        this.motorClass = objectClass;
        this.ID = ID;
        this.runToPosition = false;
        this.runUsingEncoders = false;
        return this;
    }

    public void setDirection(DcMotorSimple.Direction direction){
        this.direction = direction;
    }

    public void setMode(DcMotor.RunMode runMode){
        this.runMode = runMode;
        switch(this.runMode){
            case STOP_AND_RESET_ENCODER:
                this.runToPosition = false;
                this.power = 0f;
                this.curentPosition = 0;
                this.targetPosition = 0;
                break;

            case RUN_TO_POSITION:
                this.runToPosition = true;

            case RUN_USING_ENCODER:
                this.runUsingEncoders = true;

        }
    }
    public void setTargetPosition(int target){
        this.targetPosition = target;
    }

    public void setPower(float power){
        if(!this.runUsingEncoders){
            this.power = power;
            this.running = true;
        }
        else{
            if(this.targetPosition !=0 && this.runToPosition && this.ID!=null){
                this.power = power;
                this.running = true;
                this.y +=this.targetPosition;

            }
            else{
                this.running = false;
            }
        }
    }
}