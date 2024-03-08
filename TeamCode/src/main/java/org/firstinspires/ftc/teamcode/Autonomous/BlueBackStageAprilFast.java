/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.BACKWARDS;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold_Backstage;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold_Backstage;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold_Backstage;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;
import org.firstinspires.ftc.teamcode.Implementations.Math.MathFunc;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.IOException;


public class BlueBackStageAprilFast {
    private int PARKING=1; //-1 for left parking and 1 for right
    //  private BluePropThreshold_Backstage blueProp;
    private PIDController controller;

    public static double p=0.04, i=0, d=0.00001;//d=0.00001
    public static double f=0.002;

    public static double target;

    private final double ticks_in_degrees=288*(125/45.0)/360; /// gear ratio: 45/125=0.36

    private  Robot robot;

    private int tagID;

    private   final double ZERO_OFFSET = 70.0-3.85;
    private   double TargetPosInDegrees=70.0-3.85;


    private ElapsedTime AUTO;




    public FtcDashboard dashboard;
    private PIDController forward,strafe,turn;

    public static double Pf=0.02d, If=0d, Df=0d;
    public static double Ps=0.045d, Is=0d, Ds=0d;
    public static double Pt=0.02d, It=0.01d, Dt=0.00005d;


    private static double Targetf=0,Targets=0,Targett;

    public double val=0;



    public static double Distancef =8,Distances=6,Distancet=6;

    public static double POWER_LiMIT=0.7;

    private int hope=0;
    private Telemetry telemetry;



    public BlueBackStageAprilFast(Robot robot, Telemetry telemetry, int tagID){
        this.robot = robot;
        this.telemetry = telemetry;
        this.tagID = tagID;
    }
    public void passTag(int tagID){
        this.tagID = tagID;
    }
    public void init() {

        this.robot.camera.openFrontCam();
        target=this.robot.arm.ZERO_OFFSET;

        AUTO=new ElapsedTime();


        dashboard=FtcDashboard.getInstance();

        forward=new PIDController(Pf,If,Df);
        forward.setPID(Pf,If,Df);

        strafe=new PIDController(Ps,Is,Ds);
        strafe.setPID(Ps,Is,Ds);

        turn=new PIDController(Pt,It,Dt);
        turn.setPID(Pt,It,Dt);

        this.telemetry=new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        AUTO.reset();

        this.robot.wheels.setDirection();

    }

    public void backStageLeftProp(int parking,int timer){

        this.robot.camera.openBackCam();
    /*
        this.robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(750);
        this.robot.joint.setPosition(Joint.DOWN);
        sleep(900);
        this.robot.claw.setPosition(Claw.CLOSED);
    */
        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(750);
        this.robot.joint.setPosition(Joint.DOWN);
        sleep(900);


        this.robot.move.lateral(LEFT,0.4,25.5);
        sleep(175);

        this.robot.move.forward(FORWARD,0.6,36.5);
        sleep(175);
        this.robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(250);
        this.robot.move.forward(BACKWARDS,0.5,5.5);
        sleep(175);
        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(250);
        this.robot.joint.setPosition(Joint.UP);

        this.robot.move.rotate(1,0.6,90);
        sleep(250);

        this.robot.move.forward(BACKWARDS,0.6,30);
        this.robot.move.lateral(LEFT,0.6,15);
        this.robot.move.forward(FORWARD,0.6,10);
        this.robot.wheels.reverseDirection();

        Go_to_April();

        this.robot.wheels.setDirection();

        ElapsedTime timerr=new ElapsedTime();

        timerr.reset();


        sleep(250);

        int stateArm=0;

        boolean armtarget=false,OKtarget=false;


        while(!armtarget){

            if(AUTO.seconds()>27){

                stateArm=5;
            }

            switch (stateArm){

                case 0:

                    this.robot.arm.setPosition(ZERO_OFFSET,0.4);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=1;
                    break;

                case 1:
                    this.robot.arm.setPosition(240,0.5);
                    TargetPosInDegrees=230;
                    stateArm=2;
                    break;

                case 2:
                    if(this.robot.arm.isOnTarget(6)) {
                        stateArm=3;
                        timerr.reset();
                    }
                    break;

                case 3:

                    if(timerr.milliseconds()>250){
                        this.robot.claw.setPosition(Claw.OPEN);
                        stateArm=4;
                        timerr.reset();

                    }

                    break;

                case 4:


                    if(timerr.milliseconds()>250){

                        stateArm=5;
                    }
                    break;


                case 5:
                    this.robot.arm.setPosition(ZERO_OFFSET,0.4);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=6;
                    break;

                case 6:
                    if(this.robot.arm.getPosition()<175) {
                        OKtarget=true;
                        this.robot.arm.setPower(0);
                        armtarget=true;
                        this.robot.move.lateral(LEFT,0.8,45);
                    }
                    break;

            }

            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)<5 && OKtarget){
                this.robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.robot.arm.setPower(0);
            }
            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)>=3){
                OKtarget=false;
            }
            if(!OKtarget){
                this.robot.arm.armTask();
            }
            this.telemetry.addLine("Pos: "+this.robot.arm.getPosition());
            this.telemetry.addLine("Target: "+TargetPosInDegrees);
            this.telemetry.addLine("State: "+stateArm);
            this.telemetry.update();
        }
        /*

        robot.move.lateral(LEFT,0.6,69);

        sleep(250);

        robot.move.forward(BACKWARDS,0.6,25);

         */
    }

    public void backStageCenterProp(int parking,int timer){

        this.robot.camera.openBackCam();

        /*
        this.robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(750);
        this.robot.joint.setPosition(Joint.DOWN);
        sleep(900);
        this.robot.claw.setPosition(Claw.CLOSED);
*/
        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(750);
        this.robot.joint.setPosition(Joint.DOWN);
        sleep(900);


        this.robot.move.lateral(LEFT,0.4,8);
        sleep(175);

        this.robot.move.forward(FORWARD,0.6,57);
        sleep(175);
        this.robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);
        this.robot.move.forward(BACKWARDS,0.5,5.5);
        sleep(175);
        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(500);
        this.robot.joint.setPosition(Joint.UP);

        this.robot.move.rotate(1,0.6,90);
        sleep(250);

        //   robot.move.forward(BACKWARDS,0.6,30);

        this.robot.wheels.reverseDirection();

        Go_to_April();

        this.robot.wheels.setDirection();

        ElapsedTime timerr=new ElapsedTime();

        timerr.reset();


        sleep(250);

        int stateArm=0;

        boolean armtarget=false,OKtarget=false;


        while(!armtarget){

            if(AUTO.seconds()>27){

                stateArm=5;
            }

            switch (stateArm){

                case 0:

                    this.robot.arm.setPosition(ZERO_OFFSET,0.4);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=1;
                    break;

                case 1:
                    this.robot.arm.setPosition(240,0.5);
                    TargetPosInDegrees=230;
                    stateArm=2;
                    break;

                case 2:
                    if(this.robot.arm.isOnTarget(6)) {
                        stateArm=3;
                        timerr.reset();
                    }
                    break;

                case 3:

                    if(timerr.milliseconds()>250){
                        this.robot.claw.setPosition(Claw.OPEN);
                        stateArm=4;
                        timerr.reset();

                    }

                    break;

                case 4:


                    if(timerr.milliseconds()>250){

                        stateArm=5;
                    }
                    break;


                case 5:
                    this.robot.arm.setPosition(ZERO_OFFSET,0.4);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=6;
                    break;

                case 6:
                    if(this.robot.arm.getPosition()<175) {
                        OKtarget=true;
                        this.robot.arm.setPower(0);
                        armtarget=true;
                        this.robot.move.lateral(LEFT,0.8,45);
                    }
                    break;

            }

            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)<5 && OKtarget){
                this.robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.robot.arm.setPower(0);
            }
            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)>=3){
                OKtarget=false;
            }
            if(!OKtarget){
                this.robot.arm.armTask();
            }
            this.telemetry.addLine("Pos: "+this.robot.arm.getPosition());
            this.telemetry.addLine("Target: "+TargetPosInDegrees);
            this.telemetry.addLine("State: "+stateArm);
            this.telemetry.update();
        }
        /*

        robot.move.lateral(LEFT,0.6,58);

        sleep(250);

        robot.move.forward(BACKWARDS,0.6,25);

         */

    }
    public void backStageRightProp(int parking, double timer){

        this.robot.camera.openBackCam();

        /*
        this.robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(750);
        this.robot.joint.setPosition(Joint.DOWN);
        sleep(900);
        this.robot.claw.setPosition(Claw.CLOSED);


         */
        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(750);
        this.robot.joint.setPosition(Joint.DOWN);
        sleep(900);


        this.robot.move.forward(FORWARD,0.6,39);
        sleep(175);

        this.robot.move.rotate(1,0.6,51.5);
        sleep(500);


        this.robot.move.forward(FORWARD,0.5,10);
        sleep(500);

        this.robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(750);


        this.robot.move.forward(BACKWARDS,0.5,5.5);
        sleep(175);
        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(500);
        this.robot.joint.setPosition(Joint.UP);
        sleep(500);

        this.robot.move.rotate(1,0.6,40);
        sleep(250);

        this.robot.wheels.reverseDirection();

        Go_to_April();

        this.robot.wheels.setDirection();

        ElapsedTime timerr=new ElapsedTime();

        timerr.reset();


        sleep(250);

        int stateArm=0;

        boolean armtarget=false,OKtarget=false;


        while(!armtarget){

            if(AUTO.seconds()>27){

                stateArm=5;
            }

            switch (stateArm){

                case 0:

                    this.robot.arm.setPosition(ZERO_OFFSET,0.4);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=1;
                    break;

                case 1:
                    this.robot.arm.setPosition(240,0.5);
                    TargetPosInDegrees=230;
                    stateArm=2;
                    break;

                case 2:
                    if(this.robot.arm.isOnTarget(6)) {
                        stateArm=3;
                        timerr.reset();
                    }
                    break;

                case 3:

                    if(timerr.milliseconds()>250){
                        this.robot.claw.setPosition(Claw.OPEN);
                        stateArm=4;
                        timerr.reset();

                    }

                    break;

                case 4:


                    if(timerr.milliseconds()>250){

                        stateArm=5;
                    }
                    break;


                case 5:
                    this.robot.arm.setPosition(ZERO_OFFSET,0.4);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=6;
                    break;

                case 6:
                    if(this.robot.arm.getPosition()<175) {
                        OKtarget=true;
                        this.robot.arm.setPower(0);
                        armtarget=true;
                        this.robot.move.lateral(LEFT,0.8,45);
                    }
                    break;

            }

            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)<5 && OKtarget){
                this.robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.robot.arm.setPower(0);
            }
            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)>=3){
                OKtarget=false;
            }
            if(!OKtarget){
                this.robot.arm.armTask();
            }
            this.telemetry.addLine("Pos: "+this.robot.arm.getPosition());
            this.telemetry.addLine("Target: "+TargetPosInDegrees);
            this.telemetry.addLine("State: "+stateArm);
            this.telemetry.update();
        }

        /*
        robot.move.lateral(LEFT,0.6,40);

        sleep(250);

        robot.move.forward(BACKWARDS,0.6,25);

         */
    }


    private double ForwardPID(){


        Targetf=this.robot.move.returnRangeError(this.tagID,this.robot,this.robot.camera.atag);

        double power;


        if(Targetf==0){

            power=0;

        }else{

            power = forward.calculate(Targetf, Distancef);

        }

        return  power;


        //  robot.wheels.setPower(-power,-power,-power,-power);

    }

    private double StrafePID(){

        Targets=this.robot.move.returnYawError(this.tagID,this.robot,this.robot.camera.atag);

        // target=matee.inchToTicksD(target);



        double power;


        if(Targets==0){

            power=0;

        }else{
            double pid=strafe.calculate(Targets, Distances);

            power = pid;

        }

        return power;


        // robot.wheels.setPower(-power,power,power,-power);

    }

    private double TurnPID(){

        Targett=this.robot.move.returnHeadingError(this.tagID,this.robot,this.robot.camera.atag);

        double power;


        if(Targett==0){

            power=0;

        }else{
            double pid=turn.calculate(Targett, Distancet);

            power = pid;

        }

        return  power;

        //  robot.wheels.setPower(-power,power,-power,power);

    }

    public void AprilPID(){

        double powerForward=ForwardPID();
        double powerStrafe=StrafePID();
        double powerTurn=TurnPID();

        double powerFrontLeft=-powerForward-powerStrafe-powerTurn;
        double powerFrontRight=-powerForward+powerStrafe+powerTurn;
        double powerBackLeft=-powerForward+powerStrafe-powerTurn;
        double powerBackRight=-powerForward-powerStrafe+powerTurn;

        double maxPower=MathFunc.MaxPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);

        if(maxPower>1){

            powerFrontLeft/=maxPower;
            powerFrontRight/=maxPower;
            powerBackLeft/=maxPower;
            powerBackRight/=maxPower;
        }

        maxPower=MathFunc.MaxPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);

        if(maxPower>POWER_LiMIT){

            double coeficient= maxPower/POWER_LiMIT;

            powerFrontLeft/=coeficient;
            powerFrontRight/=coeficient;
            powerBackLeft/=coeficient;
            powerBackRight/=coeficient;
        }


        this.robot.wheels.setPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);


    }

    public void Go_to_April(){

        ElapsedTime emergency=new ElapsedTime();
        emergency.reset();

        boolean alarm=false;

        boolean done=false;
        boolean exit = false;

        while(!done){

            AprilTagDetection detection=this.robot.camera.returnAprilTAg(this.tagID);

            if(detection!=null){

                emergency.reset();

                if(detection.ftcPose.range>=Distancef-2 && detection.ftcPose.range<=Distancef+2 && detection.ftcPose.bearing>=Distancet-7 && detection.ftcPose.bearing<=Distancet+7 && detection.ftcPose.yaw>=Distances-7 && detection.ftcPose.yaw<=Distances+7){

                    hope=1;
                    done=true;
                    this.robot.wheels.setPower(0,0,0,0);

                }else if(hope==0){
                    AprilPID();

                }

                /*
                telemetry.addLine("Range: "+detection.ftcPose.range);
                telemetry.addLine("Bearing: "+detection.ftcPose.bearing);
                telemetry.addLine("Yaw: "+detection.ftcPose.yaw);


                 */

                //  telemetry.update();

            }else{

                if(emergency.seconds()>5){

                    done=true;
                    alarm=true;
                    exit = true;

                }

                this.robot.wheels.setPower(0,0,0,0);


            }

        }

        if(alarm){

            this.robot.move.lateral(RIGHT,0.8,45);

            while(exit){

            }

        }


    }
}

//759 lines and 40 warnings 331 lines and 27 warnings