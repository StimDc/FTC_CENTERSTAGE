package org.firstinspires.ftc.teamcode.ControlPart;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Implementations.DebugTools.CatchingBugs;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.Experimental;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.ImplementedBy;

@TeleOp(name = "Pentru Sponsori")
public class Sponsori extends OpMode {
    private DcMotor frontLeft, frontRight,backLeft, backRight;
    private DcMotor slider;
    private float leftX, leftY, rightX;
    private static final float deadZone = 0.4f;
    private Servo c1,c2;
    @Override
    public void init() {

        frontLeft = hardwareMap.get(DcMotor.class , "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);


        slider = hardwareMap.get(DcMotor.class, "SL");

        slider.setDirection(DcMotorSimple.Direction.REVERSE);

        c1= hardwareMap.get(Servo.class, "C1");
        c2 = hardwareMap.get(Servo.class, "C2");

    }

    @Experimental()
    @Override
    public void loop() {
        leftY = gamepad1.left_stick_y;
        leftX = gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;

        if(abs(leftY) >deadZone && abs(leftY) > abs(leftX)){
            setPowerWheelMotor(leftY,1,1,1,1);
        }

        if(abs(gamepad1.left_stick_x) > deadZone && abs(gamepad1.left_stick_x) > abs(gamepad1.left_stick_y)){
            setPowerWheelMotor(leftX,-1,1,+1,-1);

        }

        if(abs(gamepad1.right_stick_x) > deadZone){
            setPowerWheelMotor(rightX,-1,1,-1,1);

        }

        if(gamepad1.a){
            setPowerSliderMotor(0.7);
        }
        if(gamepad1.b){
            setPowerSliderMotor(-0.5);
        }
        //TODO: de modificat pozitiile
        //c1- servo din stanga: 1-deschis la maxim, 0-inchis la maxim
        //c2-servo di dreapta: 1-inchis la maxim, 0-deschis la maxim

        if(gamepad1.x){//inchide
            c1.setPosition(0.45);
            c2.setPosition(0.53);
        }
        if(gamepad1.y){ //deschide
            c1.setPosition(0.55);
            c2.setPosition(0.38);
        }
        if(slider.getPower()==0) {
            slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        stopMotor();
        slider.setPower(0);
        debug();

    }


    /**
     * Sets the power of the motor attached to the slider
     * @param power is the power that feeds the motor, 0 represents 0V, 1 means MAX V and -1 means the same but in reverse
     */
    @Experimental
    @ImplementedBy(name = "Andrei", date="18.09.23")
    public void setPowerSliderMotor(double power){
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        for(float i = 0;i< abs(power);i+=0.05){
            slider.setPower(i * power/abs(power));
        }
    }
    /**
     * Sets the power of the motors attached to the wheels
     * @param power is the power that feeds the motor 0 represents 0V, 1 means MAX V and -1 means MAX V but in reverse
     * @param signFrontLeft tells if the power fed to the front left motor is positive (+1) or negative(-1 )
     * @param signFrontRight tells if the power fed to the front right motor is positive (+1) or negative(-1 )
     * @param signBackLeft tells if the power fed to the back left motor is positive (+1) or negative(-1 )
     * @param signBackRight tells if the power fed to the back right motor is positive (+1) or negative(-1 )
     */
    //@Experimental
    @ImplementedBy(name = "Andrei", date="18.09.23")
    public void setPowerWheelMotor(float power, int signFrontLeft, int signFrontRight, int signBackLeft, int signBackRight){
        frontLeft.setPower(signFrontLeft *power);
        frontRight.setPower(signFrontRight * power);
        backLeft.setPower(signBackLeft * power);
        backRight.setPower(signBackRight * power);
    }


    @Experimental
    @ImplementedBy(name = "Andrei", date="18.09.23")
    public void stopMotor(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    @Experimental
    @ImplementedBy(name = "Andrei", date="18.09.23")
    public void debug(){
        telemetry.addLine("DEBUG");
        CatchingBugs.getExperimental(telemetry,this.getClass());
        CatchingBugs.getNameReport(telemetry,"Andrei");
        debugGamepad(gamepad1);
        telemetry.update();
    }

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
