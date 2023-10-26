package org.firstinspires.ftc.teamcode.ControlPart;

import static java.lang.Math.abs;

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

@TeleOp(name = "Test")
public class Test extends OpMode {
    private float leftX, leftY, rightX;

    private final double COEFICIENT= 0.0001d;

    private double OK=0;

    static final double C2MINANGLE=0.52988888888d;
    static final double C2MAXANGLE=0.34999999999d;
    static final double C1MINANGLE=0.0296666d;
    static final double C1MAXANGLE=0.17333333d;

    static final double POWER=0.7d;

    private DcMotor         leftDrive   = null, rightDrive=null;

    private Servo c1,c2;

    @Override
    public void init() {


        c1= hardwareMap.get(Servo.class, "c1");
        c2 = hardwareMap.get(Servo.class, "c2");
        leftDrive= hardwareMap.get(DcMotor.class,"motor1");
        rightDrive= hardwareMap.get(DcMotor.class,"motor2");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


    }

    @Experimental()
    @Override
    public void loop() {
        if(OK==0){
            c1.setPosition(C1MINANGLE);
            c2.setPosition(C2MINANGLE);
            OK=1;
        }
        leftY = gamepad1.left_stick_y;
        leftX = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;

        if(gamepad1.a){
            leftDrive.setPower(POWER);
            rightDrive.setPower(POWER);
        }else{
            leftDrive.setPower(0);
            rightDrive.setPower(0);
        }

        if(gamepad1.x){//inchide
            c1.setPosition(C1MAXANGLE);
            c2.setPosition(C2MAXANGLE);
        }
        if(gamepad1.y){ //deschide
            c1.setPosition(C1MINANGLE);
            c2.setPosition(C2MINANGLE);
        }
        telemetry.addData("Servo Position: ", c1.getPosition());
        telemetry.update();

    }


    /**
     * Sets the power of the motor attached to the slider
     * @param power is the power that feeds the motor, 0 represents 0V, 1 means MAX V and -1 means the same but in reverse
     */
    @Experimental
    @ImplementedBy(name = "Andrei", date="18.09.23")

    /**
     * Sets the power of the motors attached to the wheels
     * @param power is the power that feeds the motor 0 represents 0V, 1 means MAX V and -1 means MAX V but in reverse
     * @param signFrontLeft tells if the power fed to the front left motor is positive (+1) or negative(-1 )
     * @param signFrontRight tells if the power fed to the front right motor is positive (+1) or negative(-1 )
     * @param signBackLeft tells if the power fed to the back left motor is positive (+1) or negative(-1 )
     * @param signBackRight tells if the power fed to the back right motor is positive (+1) or negative(-1 )
     */


    public void debugGamepad(Gamepad gamepad){
        telemetry.addLine("Gamepad debug");
        telemetry.addLine("Left Stick X : " + gamepad.left_stick_x);
        telemetry.addLine("Left Stick Y : " + gamepad.left_stick_y);
        telemetry.addLine("Right Stick X : " + gamepad.right_stick_x);
        telemetry.addLine("Right Stick Y : " + gamepad.right_stick_y);

        String dpad, button;
        button = (gamepad.a)? "A" : ((gamepad.b)? "B" : ((gamepad.y)? "Y" : ((gamepad.x)? "X" : "NONE")));
        dpad = (gamepad.dpad_down)? "DOWN" : ((gamepad.dpad_left)? "LEFT" : ((gamepad.dpad_right)? "RIGHT" : ((gamepad.dpad_up)? "UP" : "NONE")));

        telemetry.addLine("Gamepad dpad :" + dpad);
        telemetry.addLine("Gamepad button : " + button);
    }

}
