package org.firstinspires.ftc.teamcode.Implementations.DebugTools;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Servo Calibrare")

public class ServoCalibrare extends OpMode {

    private Servo test;
    private double angle=0.5d;

    @Override
    public void init() {

    test=hardwareMap.get(Servo.class,"hl");

    }

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
