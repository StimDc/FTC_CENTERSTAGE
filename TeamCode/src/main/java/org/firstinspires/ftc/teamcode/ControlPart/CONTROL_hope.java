package org.firstinspires.ftc.teamcode.ControlPart;

import static java.lang.Math.abs;
import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
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

@Config
@TeleOp(name = "Hope Control")

public class CONTROL_hope extends OpMode {

    private Servo joint, claw;
    private boolean OKJoint=true,OKClaw=true, PressClaw=false, PressJoint=false;

    private boolean once=false;

    static final double CLAWCLOSED=0.393888d, CLAWOPEN=0.1d, CLAWINTERMEDIARY=0.344444d;
    static final double JOINTUP=1, JOINTDOWN=0.119333d;

    static final double AVIONSTART=0, AVIONRELEASE=0.5d;

    private DcMotorEx  elevator1, elevator2;

    private DcMotor frontLeft,frontRight, backLeft, backRight;

 ///PID ARM

    private PIDController controller;

    public static double p=0.03, i=0, d=0.00001;
    public static double f=0.05;

    public static int target=0;

    public double val=0;

    private final double ticks_in_degrees=288/(360.0*0.36); /// gear ratio: 45/125=0.36


    @Override
    public void init() {

        controller=new PIDController(p,i,d);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


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

        elevator1=hardwareMap.get(DcMotorEx.class,"e1");
        elevator2=hardwareMap.get(DcMotorEx.class,"e2");

        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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

        double drive =0;
        double strafe=0;
        double turn =0;

      /*NORMAL METHOD
        drive=-gamepad1.left_stick_y/2.0;
        strafe=-gamepad1.left_stick_x/2.0;
        turn=-gamepad1.right_stick_x/3.0;
       */

        ///TRYING METHOD
        drive=-gamepad1.left_stick_y/1.5;
        strafe=-gamepad1.left_stick_x*1.1/1.5;
        turn=-gamepad1.right_stick_x/1.5;

        moveRobot(drive,strafe,turn);
        moveElevator();

    }

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

    public void moveElevator(){

        controller.setPID(p,i,d);
        int elepos=elevator1.getCurrentPosition();

        double pid=controller.calculate(elepos, target);
        double ff=Math.cos(Math.toRadians(target/ticks_in_degrees))*f;

        double power = pid + ff;

        elevator1.setPower(power);
        elevator2.setPower(power);

        telemetry.addData("pos ",elepos);
        telemetry.addData("target ", target);
        telemetry.update();

        if(gamepad2.left_trigger>0){
            val+=gamepad2.left_trigger;
        }

        if(gamepad2.right_trigger>0 && val>0){
            val-=gamepad2.right_trigger;
        }

        if(target==0){
            elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        target=(int) val;

    }

    }


