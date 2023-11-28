package org.firstinspires.ftc.teamcode.ControlPart;

import static java.lang.Math.abs;
import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Implementations.DebugTools.CatchingBugs;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.Experimental;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.ImplementedBy;

@TeleOp(name = "Servo Calibrare")

public class ServoCalibrare extends OpMode {

    private Servo test;
    private double angle=0.5d;

    @Override
    public void init() {

    test=hardwareMap.get(Servo.class,"claw");

    }


    @Experimental()
    @Override
    public void loop() {

        if(gamepad1.a){
            test.setPosition(0);
        }

        if(gamepad1.b){
            test.setPosition(1);
        }

        if(gamepad1.x){
            test.setPosition(angle);
            angle+=0.0001;
        }

        if(gamepad1.y){
            test.setPosition(angle);
            angle-=0.0001;
        }

        telemetry.addData("Position",test.getPosition());
        telemetry.update();

    }

}
