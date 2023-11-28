package org.firstinspires.ftc.teamcode.ControlPart;

import static android.os.SystemClock.sleep;
import static java.lang.Math.abs;

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

@TeleOp(name = "Servo pls")

public class Servo_pls extends OpMode {

    private Servo joint, claw;
    private boolean OKJoint=true,OKClaw=true, PressClaw=false, PressJoint=false;

    private boolean once=false;

    static final double CLAWCLOSED=0.393888d, CLAWOPEN=0.1d, CLAWINTERMEDIARY=0.344444d;
    static final double JOINTUP=1, JOINTDOWN=0.1213333d;

    static final double AVIONSTART=0, AVIONRELEASE=0.5d;

    private DcMotor elevator1, elevator2;

    private DcMotor frontLeft,frontRight, backLeft, backRight;

    @Override
    public void init() {

        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        claw=hardwareMap.get(Servo.class,"claw");
        joint=hardwareMap.get(Servo.class,"joint");

        elevator1=hardwareMap.get(DcMotor.class,"e1");
        elevator2=hardwareMap.get(DcMotor.class,"e2");

        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    @Experimental()
    @Override
    public void loop() {

        if(!once){
            joint.setPosition(JOINTUP);
            claw.setPosition(CLAWINTERMEDIARY);
            once=true;
        }

        if(gamepad2.b){
            if(OKJoint==false && !PressJoint){
                joint.setPosition(JOINTUP);
                OKJoint=true;

            }
           else if(OKJoint==true && !PressJoint){
                joint.setPosition(JOINTDOWN);
                OKJoint=false;

            }
            PressJoint=true;

        }else {
            PressJoint=false;
        }


        if(gamepad2.dpad_right){
            joint.setPosition(0.4);
            OKJoint=false;
        }

        if(gamepad2.dpad_left){
            claw.setPosition(CLAWOPEN);
            OKClaw=false;
        }

        if(gamepad2.a){
            if(OKClaw==false && !PressClaw){
                claw.setPosition(CLAWINTERMEDIARY);
                OKClaw=true;

            }
            else if(OKClaw==true && !PressClaw){
                claw.setPosition(CLAWCLOSED);
                OKClaw=false;

            }
            PressClaw=true;
        }else{
            PressClaw=false;
        }



        if(gamepad2.left_trigger>0){ //The elevator is going up

            if(gamepad2.left_trigger>0.65){ ///SISTEM ANTI PROST
                UpElevator(0.65);
            }else{
                UpElevator(gamepad2.left_trigger);
            }

        }else if(gamepad2.right_trigger>0){//The elevator is going down

            if(gamepad2.right_trigger>0.65){///SISTEM ANTI PROST HAI ACASA
                DownElevator(0.65);
            }else{
                DownElevator(gamepad2.right_trigger);
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
            powerWheelMotors(gamepad1.left_trigger,0,1,0,1);

        }

        if(gamepad1.right_trigger >0){
            powerWheelMotors(gamepad1.right_trigger,1,0,1,0);
        }

        if(gamepad1.left_stick_x >0.4 || gamepad1.left_stick_x <-0.4){
            powerWheelMotors(gamepad1.left_stick_x, -1,1,1,-1);
        }


        stop1();

         */

        ///NEW METHOD FOR CHASIS MOVEMENT

        double drive =0;
        double strafe=0;
        double turn =0;

        drive=-gamepad1.left_stick_y/2.0;
        strafe=-gamepad1.left_stick_x/2.0;
        turn=-gamepad1.right_stick_x/3.0;

        moveRobot(drive,strafe,turn);

    }


    ///FOR THE NEW MOVEMENT OF CHASIS
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }


    ///FOR THE OLD MOVEMENT CODE
    public void stop1(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

    }


    ///FOR THE OLD MOVEMENT CODE
    public void powerWheelMotors(float trip, int sign1, int sign2, int sign3, int sign4){

        frontRight.setPower(sign1 * trip);
        frontLeft.setPower(sign2 * trip);
        backRight.setPower(sign3* trip);
        backLeft.setPower(sign4* trip);
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

}


