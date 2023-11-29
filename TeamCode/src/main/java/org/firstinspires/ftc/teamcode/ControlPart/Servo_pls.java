package org.firstinspires.ftc.teamcode.ControlPart;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.Experimental;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.ImplementedBy;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Wheels;

@TeleOp(name = "Servo pls")

public class Servo_pls extends OpMode {

    private Servo joint, claw;
    private boolean OKJoint=true,OKClaw=true, PressClaw=false, PressJoint=false;

    private boolean once=false;


    private DcMotor elevator1, elevator2;

    private Wheels wheels;

    @Override
    public void init() {

        wheels = new Wheels(hardwareMap);
        wheels.setDirection();



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
            joint.setPosition(Joint.UP);
            claw.setPosition(Claw.INTERMEDIARY);
            once=true;
        }

        if(gamepad2.b){
            if(!OKJoint && !PressJoint){
                joint.setPosition(Joint.UP);
                OKJoint=true;

            }
           else if(OKJoint && !PressJoint){
                joint.setPosition(Joint.DOWN);
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
            claw.setPosition(Claw.OPEN);
            OKClaw=false;
        }

        if(gamepad2.a){
            if(!OKClaw && !PressClaw){
                claw.setPosition(Claw.INTERMEDIARY);
                OKClaw=true;

            }
            else if(OKClaw && !PressClaw){
                claw.setPosition(Claw.CLOSED);
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
        

        double drive;
        double strafe;
        double turn;

        drive=-gamepad1.left_stick_y/2.0;
        strafe=-gamepad1.left_stick_x/2.0;
        turn=-gamepad1.right_stick_x/3.0;

        moveRobot(drive,strafe,turn);

    }


    ///FOR THE NEW MOVEMENT OF CHASIS
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower = x - y - yaw;
        double frontRightPower = x + y + yaw;
        double backLeftPower = x + y - yaw;
        double backRightPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        wheels.setPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
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


