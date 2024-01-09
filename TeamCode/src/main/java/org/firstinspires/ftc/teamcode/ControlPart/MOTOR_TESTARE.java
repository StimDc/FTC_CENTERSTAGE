package org.firstinspires.ftc.teamcode.Implementations.DebugTools;

import static java.lang.Math.abs;
import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Implementations.DebugTools.CatchingBugs;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.Experimental;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.ImplementedBy;

@TeleOp(name = "MOTOR TESTARE")

public class MOTOR_TESTARE extends OpMode {

    DcMotor testare;

    @Override
    public void init() {

        testare=hardwareMap.get(DcMotor.class,"v");

        testare.setDirection(DcMotorSimple.Direction.FORWARD);

    }


    @Experimental()
    @Override
    public void loop() {

        if(gamepad1.a){

            testare.setPower(1);

        }else if(gamepad1.b){

            testare.setPower(-1);

        }else{

            testare.setPower(0);

        }


    }

}
