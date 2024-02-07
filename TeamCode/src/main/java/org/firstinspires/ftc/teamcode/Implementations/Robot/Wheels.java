package org.firstinspires.ftc.teamcode.Implementations.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
        this.frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        this.frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        this.backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        this.backRight.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void reverseDirection(){
        this.frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        this.frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        this.backRight.setDirection(DcMotorEx.Direction.REVERSE);
    }



    public void GetDirection(Telemetry telemetry){

        telemetry.addLine("Front Left: "+frontLeft.getDirection());
        telemetry.addLine("Front Right: "+frontRight.getDirection());
        telemetry.addLine("Back Left: "+backLeft.getDirection());
        telemetry.addLine("Back Right: "+backRight.getDirection());


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