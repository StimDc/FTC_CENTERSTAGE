package org.firstinspires.ftc.teamcode.Implementations.DebugTools;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Debugger {
    DcMotorDebug frontLeft, frontRight, backLeft,backRight;

    Debugger(){
        this.frontLeft = new DcMotorDebug();
        this.frontRight = new DcMotorDebug();
        this.backLeft = new DcMotorDebug();
        this.backRight = new DcMotorDebug();
    }
    public void init(){
        frontLeft = frontLeft.hardwareMap(DcMotor.class, "FL");
        frontRight = frontRight.hardwareMap(DcMotor.class, "FR");
        backLeft = frontRight.hardwareMap(DcMotor.class, "BL");
        backRight = backRight.hardwareMap(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    public void loop(){
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setTargetPosition(1000);
        frontLeft.setPower(0.5f);
        displayText(frontLeft);
    }
    public void displayText(DcMotorDebug motor){
        System.out.println("MOTOR : " + motor.ID);
        System.out.println("Direction : " + motor.direction);
        System.out.println("RunMode : " + motor.runMode);
        System.out.println("Power : " + motor.power);
        if(motor.runUsingEncoders){
            System.out.println("Target Position : " + motor.targetPosition);
            System.out.println("Curent Position : " + motor.curentPosition);
        }
        System.out.println("Current Position : X:" + motor.x + " Y:" + motor.y);

    }
}
