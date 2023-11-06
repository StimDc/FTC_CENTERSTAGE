package org.firstinspires.ftc.teamcode.ControlPart;

import static android.os.SystemClock.sleep;
import static java.lang.Math.abs;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.DebugTools.CatchingBugs;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.Experimental;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.ImplementedBy;

@TeleOp(name = "Control YAYY")
public class Control extends OpMode {
    ///private float leftX, leftY, rightX;

    private final double COEFICIENT= 0.001d;

    private double angle=0;


    static final double C0INCHIS=0.3511111d;
    static final double C0DESCHIS=0.15166666d;

     boolean OKDeschis=true,  OKSus=true;
    private double PowerElevator=0.7d, PowerClaw=0.2d;

    static final double TICKS_REVHEX=288;

    private final double GEAR_RATIO_CLAW=2; //90teeth:45teeth

     DcMotor frontLeft,frontRight, backLeft, backRight,viteza;

    DcMotor claw;
     DcMotor elevator1, elevator2;

     private double elepower;

     public ServoEx testservo;

    private Servo c0;

    private CRServo sgheara;

    @Override
    public void init() {

       /* frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");


        */
        c0= hardwareMap.get(Servo.class, "c0");
        ///claw=hardwareMap.get(DcMotor.class,"gheara");
        ///sgheara=hardwareMap.get(CRServo.class, "sgheara");
       //// testservo=hardwareMap.get(Servo.class,"testservo");
       // testservo=hardwareMap.get(SimpleServo.class,"testservo");
        testservo = new SimpleServo(hardwareMap,"testservo",0,180,AngleUnit.DEGREES);
        testservo.setRange(0,300, AngleUnit.DEGREES);


        viteza=hardwareMap.get(DcMotor.class,"viteza");

        elevator1=hardwareMap.get(DcMotor.class,"ele1");
        elevator2=hardwareMap.get(DcMotor.class,"ele2");

        claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw.setDirection(DcMotorSimple.Direction.FORWARD);

        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);

        //elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       /*
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        */

    }

    @Experimental()
    @Override
    public void loop() {

        claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        if(gamepad1.dpad_up){
            OKDeschis=!OKDeschis;
            sleep(300);
        }
        if(OKDeschis==true){
            OpenClaw();
        }else if(OKDeschis==false){
            CloseClaw();
        }

        if(gamepad1.a){
               // testservo.rotateByAngle(250);
            ///testservo.rotateByAngle(200);
            c0.setPosition(0);
        }

        if(gamepad1.b){
         //   testservo.setPosition(1);
            c0.setPosition(1);
        }

        if(gamepad1.x){
            viteza.setPower(0.95);
        }else{
            viteza.setPower(0);
        }


       /* if(gamepad1.a){ // the claw is going down
            sleep(300);
            DownClaw(PowerClaw);
        }else if(gamepad1.b){ //the claw is going up
            sleep(300);
            UpClaw(PowerClaw);
        }

*/
        if(gamepad1.left_trigger>0){ //The elevator is going up

            if(gamepad1.left_trigger>0.7){ ///SISTEM ANTI PROST
                UpElevator(0.7);
            }else{
                UpElevator(gamepad1.left_trigger);
            }

        }else if(gamepad1.right_trigger>0){//The elevator is going down

            if(gamepad1.right_trigger>0.7){///SISTEM ANTI PROST HAI ACASA
                DownElevator(0.7);
            }else{
                DownElevator(gamepad1.right_trigger);
            }

        }else{
            elevator1.setPower(0);
            elevator2.setPower(0);


        }

        /*
        if(gamepad1.left_stick_y >0.4 || gamepad1.left_stick_y <-0.4){
            powerWheelMotors(gamepad1.left_stick_y, -1,-1,-1,-1);
        }

        if(gamepad1.right_stick_x !=0){
            powerWheelMotors(gamepad1.right_stick_x,-1,1,-1,1);
        }

        if(gamepad1.left_trigger >0){
            powerWheelMotors(gamepad1.left_trigger,1,-1,-1,1);

        }

        if(gamepad1.right_trigger >0){
            powerWheelMotors(gamepad1.right_trigger,-1,1,1,-1);
        }

         */
/*
        if(gamepad1.left_bumper){
            frontRight.setPower(0.6);
            frontLeft.setPower(-0.6);
            backRight.setPower(0.6);
            backLeft.setPower(-0.6);
        }
        if(gamepad1.right_bumper){
            frontRight.setPower(-0.6);
            frontLeft.setPower(0.6);
            backRight.setPower(-0.6);
            backLeft.setPower(0.6);
        }


        stop();
        */
        telemetry.addData("Servo Position: ", c0.getPosition());
        telemetry.update();

    }


    public void stop(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

    }
    public void powerWheelMotors(float trip, int sign1, int sign2, int sign3, int sign4){

        frontRight.setPower(sign1 * trip);
        frontLeft.setPower(sign2 * trip);
        backRight.setPower(sign3* trip);
        backLeft.setPower(sign4* trip);
    }

    /**
     * Opens the claw
     */
    @ImplementedBy(name= "Rares", date= "24.10.23")
    public void OpenClaw(){
        c0.setPosition(C0DESCHIS);
    }

    /**
     * Closes the claw
     */
    @ImplementedBy(name= "Rares",date="24.10.23")
    public void CloseClaw(){
        c0.setPosition(C0INCHIS);
    }


    /**
     * Moves the claw up or dawn
     * @param power - the power of the motor
     * @param sign - 1 means that the claw is moving upwards, and -1, downwards
     */

    @ImplementedBy(name= "Rares",date="24.10.23")
    public void MoveClaw(double spins, double power,int sign){
        claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int TICKStoRUN= (int)(TICKS_REVHEX*GEAR_RATIO_CLAW*spins);
        claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ///claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw.setTargetPosition(TICKStoRUN);//0.663
        claw.setPower(sign*power);
        claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(claw.isBusy()){
            /*if(claw.getTargetPosition()-claw.getCurrentPosition()==75){
                for(double e = claw.getPower();e>=0.1;e-=0.05){
                    claw.setPower(e);
                    sleep(200);
                }
            }

             */
        }

        sleep(200);
        claw.setPower(0);
        claw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Moves the claw upwards
     ** @param power is the power that feeds the motor, 0 represents 0V, 1 means MAX V and -1 means the same but in reverse
     */

    @ImplementedBy(name= "Rares",date="24.10.23")
    public void UpClaw(double power){

        //MoveClaw(power,-1);
        claw.setDirection(DcMotorSimple.Direction.REVERSE);
        MoveClaw(0.35,power,1);
        claw.setDirection((DcMotorSimple.Direction.FORWARD));
    }

    /**
     *Moves the claw downwards
     ** @param power is the power that feeds the motor, 0 represents 0V, 1 means MAX V and -1 means the same but in reverse
     */
    @ImplementedBy(name= "Rares",date="24.10.23")
    public void DownClaw(double power){
        MoveClaw(0.35,power,1);
    }

    /**
     * Moves the whole arm, up or down
     * @param power is the power that feeds the motor, 0 represents 0V, 1 means MAX V and -1 means the same but in reverse
     * @param sign the both motors are moving in the same direction. 1- means up, -1 means down
     */
    @ImplementedBy(name= "Rares",date="24.10.23")
    public void MoveElevator(double power,int sign){
        elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        elevator1.setPower(sign*power);
        elevator2.setPower(sign*power);

        elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Moves the arm up
     * @param power is the power that feeds the motor, 0 represents 0V, 1 means MAX V and -1 means the same but in reverse
     */
    @ImplementedBy(name= "Rares",date="24.10.23")
    public void UpElevator(double power){
        MoveElevator(power,1);
    }

    /**
     * Moves the arm down
     * @param power is the power that feeds the motor, 0 represents 0V, 1 means MAX V and -1 means the same but in reverse
     */
    @ImplementedBy(name= "Rares",date="24.10.23")
    public void DownElevator(double power){
        elevator1.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator2.setDirection(DcMotorSimple.Direction.FORWARD);

        MoveElevator(power,1);

        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

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
