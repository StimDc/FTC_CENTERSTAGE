package org.firstinspires.ftc.teamcode.Implementations.Robot;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.teamcode.Implementations.Constants.UniversalConsts.TICKS_IN_DEGREES;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Implementations.Constants.PIDConstantsArm;

public class Arm {
    public PIDController controller;
    public DcMotorEx elevator1, elevator2;
    public Telemetry telemetry;
    private static int target = 3;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry){
        this.controller = new PIDController(PIDConstantsArm.p,PIDConstantsArm.i,PIDConstantsArm.d);
        this.telemetry =new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.elevator1 = hardwareMap.get(DcMotorEx.class, "e1");
        this.elevator2 = hardwareMap.get(DcMotorEx.class, "e2");

        this.setDirection(DcMotorSimple.Direction.REVERSE);
        this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMode(DcMotor.RunMode mode){
        elevator1.setMode(mode);
        elevator2.setMode(mode);
    }

    public void setDirection(DcMotorSimple.Direction direction){
        elevator1.setDirection(direction);
        elevator2.setDirection(direction);
    }

    public void setPower(double power){
        elevator1.setPower(power);
        elevator2.setPower(power);
    }
    public void moveArm(int desiredTarget){//TODO: RESCRIS

        if(elevator1.getCurrentPosition()<desiredTarget){

            for(target = elevator1.getCurrentPosition();target<=desiredTarget;target++){
                controller.setPID(PIDConstantsArm.p, PIDConstantsArm.i, PIDConstantsArm.d);
                int elepos = elevator1.getCurrentPosition();
                double pid = controller.calculate(elepos, target);
                double ff = Math.cos(Math.toRadians(target / TICKS_IN_DEGREES));

                double power = pid + ff;

                elevator1.setPower(power);
                elevator2.setPower(power);
                sleep(250);
            }

        }else if(elevator1.getCurrentPosition()>desiredTarget){

            for(target = elevator1.getCurrentPosition();target>=0;target--){
                controller.setPID(PIDConstantsArm.p, PIDConstantsArm.i, PIDConstantsArm.d);
                int elepos = elevator1.getCurrentPosition();
                double pid = controller.calculate(elepos, target);
                double ff = Math.cos(Math.toRadians(target / TICKS_IN_DEGREES));

                double power = pid + ff;

                elevator1.setPower(power);
                elevator2.setPower(power);
                sleep(250);
            }
        }
    }
}
