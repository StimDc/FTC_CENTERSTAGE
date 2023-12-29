package org.firstinspires.ftc.teamcode.Implementations.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Implementations.Annotations.ImplementedBy;
import org.firstinspires.ftc.teamcode.Implementations.Math.MathFunc;

public class Wheels {


    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;


    public Wheels(HardwareMap hardwareMap){
        this.frontLeft = hardwareMap.get(DcMotor.class, "FL");
        this.frontRight = hardwareMap.get(DcMotor.class, "FR");
        this.backLeft = hardwareMap.get(DcMotor.class, "BL");
        this.backRight = hardwareMap.get(DcMotor.class, "BR");
    }

    public void setDirection(){
        this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower){
        this.frontLeft.setPower(frontLeftPower);
        this.frontRight.setPower(frontRightPower);
        this.backLeft.setPower(backLeftPower);
        this.backRight.setPower(backRightPower);
    }
    public void setPower(double power){
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        this.backRight.setPower(power);
    }

    public void setMode(DcMotor.RunMode mode){
        this.frontLeft.setMode(mode);
        this.frontRight.setMode(mode);
        this.backLeft.setMode(mode);
        this.backRight.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        this.frontLeft.setZeroPowerBehavior(behavior);
        this.frontRight.setZeroPowerBehavior(behavior);
        this.backLeft.setZeroPowerBehavior(behavior);
        this.backRight.setZeroPowerBehavior(behavior);
    }

    public void setTargetPosition(int frontLeftTarget, int frontRightTarget,int backLeftTarget,int backRightTarget){
        this.frontLeft.setTargetPosition(frontLeftTarget);
        this.frontRight.setTargetPosition(frontRightTarget);
        this.backLeft.setTargetPosition(backLeftTarget);
        this.backRight.setTargetPosition(backRightTarget);
    }

    public void waitMotors(){
        while(this.frontRight.isBusy() && this.frontLeft.isBusy() && this.backRight.isBusy() && this.backLeft.isBusy()){

        }
    }

    public boolean isDistNotReached(double dist){
        return Math.abs(this.frontLeft.getCurrentPosition())<Math.abs(dist)&&
                Math.abs(this.frontRight.getCurrentPosition())<Math.abs(dist) &&
                Math.abs(this.backRight.getCurrentPosition())<Math.abs(dist) &&
                Math.abs(this.backLeft.getCurrentPosition())<Math.abs(dist);
    }

}
