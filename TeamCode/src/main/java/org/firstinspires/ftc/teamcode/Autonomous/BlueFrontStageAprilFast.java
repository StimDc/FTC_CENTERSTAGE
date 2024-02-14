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

import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;
import org.firstinspires.ftc.teamcode.Implementations.Math.MathFunc;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.IOException;

@Autonomous(name="BLUE FRONTSTAGE April FastE", group = "Blue Routes")

public class BlueFrontStageAprilFast extends  LinearOpMode{

    private int PARKING=1; //-1 for left parking and 1 for right
    //  private BluePropThreshold_Backstage blueProp;
    private PIDController controller;

    public static double p=0.04, i=0, d=0.00001;//d=0.00001
    public static double f=0.002;

    public static double target;

    private final double ticks_in_degrees=288*(125/45.0)/360; /// gear ratio: 45/125=0.36

    private DcMotorEx elevator1, elevator2;
    private  Robot robot;

    private int tagID;

    private   final double ZERO_OFFSET = 70.0-3.85;
    private   double TargetPosInDegrees=70.0-3.85;


    private ElapsedTime AUTO;





    public MathFunc mate;


    public FtcDashboard dashboard;
    private PIDController forward,strafe,turn;

    public static double Pf=0.02d, If=0d, Df=0d;
    public static double Ps=0.045d, Is=0d, Ds=0d;
    public static double Pt=0.02d, It=0.01d, Dt=0.00005d;


    private static double Targetf=0,Targets=0,Targett;

    public double val=0;



    public static double Distancef =8,Distances=0,Distancet=0;//-10

    public static double POWER_LiMIT=0.7;//0.7

    private int hope=0;


    @Override
    public void runOpMode () {

        // blueProp=new BluePropThreshold_Backstage();
        try {
            robot = new Robot(hardwareMap,telemetry);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        robot.camera.openFrontCam();
        target=robot.arm.ZERO_OFFSET;

        AUTO=new ElapsedTime();


        dashboard=FtcDashboard.getInstance();

        forward=new PIDController(Pf,If,Df);
        forward.setPID(Pf,If,Df);

        strafe=new PIDController(Ps,Is,Ds);
        strafe.setPID(Ps,Is,Ds);

        turn=new PIDController(Pt,It,Dt);
        turn.setPID(Pt,It,Dt);

        telemetry=new MultipleTelemetry(telemetry, dashboard.getTelemetry());




        String propPosition=robot.camera.GetPropPosition();

        telemetry.addLine("Prop: "+propPosition);
        telemetry.update();

        //String propPosition="nope";


        boolean once=true;

        waitForStart();
        AUTO.reset();

        robot.wheels.setDirection();


        while ((propPosition.equals("nope") || once) && opModeIsActive() && !isStopRequested()){

            telemetry.addLine("Nope :( "+propPosition);

            propPosition=robot.camera.GetPropPosition();
            // propPosition="center";

            if(propPosition.equals("left")){

                telemetry.addLine(propPosition);
                telemetry.update();
                tagID=1;
                once=false;

                Backstage_LeftProp_Red(PARKING,0);

            }else if(propPosition.equals("center")){

                telemetry.addLine(propPosition);
                telemetry.update();
                tagID=2;
                once=false;

                Frontstage_CenterProp_Blue(PARKING,0);

            }else if(propPosition.equals("right")){

                telemetry.addLine(propPosition);
                telemetry.update();
                tagID=3;
                once=false;

                Backstage_RightProp_Red(PARKING,0);

            }

            telemetry.update();

        }




    }

    public void Backstage_LeftProp_Red(int parking,int timer){

        robot.camera.openBackCam();

        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(750);
        robot.joint.setPosition(Joint.DOWN);
        sleep(900);
        robot.claw.setPosition(Claw.CLOSED);



        robot.move.lateral(LEFT,0.4,25.5);
        sleep(175);

        robot.move.forward(FORWARD,0.6,36.5);
        sleep(175);
        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(250);
        robot.move.forward(BACKWARDS,0.5,5.5);
        sleep(175);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(250);
        robot.joint.setPosition(Joint.UP);

        robot.move.rotate(1,0.6,90);
        sleep(250);

        robot.wheels.reverseDirection();

        Go_to_April();

        robot.wheels.setDirection();

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

                    robot.arm.setPosition(ZERO_OFFSET,0.4);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=1;
                    break;

                case 1:
                    robot.arm.setPosition(240,0.5);
                    TargetPosInDegrees=230;
                    stateArm=2;
                    break;

                case 2:
                    if(robot.arm.isOnTarget(6)) {
                        stateArm=3;
                        timerr.reset();
                    }
                    break;

                case 3:

                    if(timerr.milliseconds()>250){
                        robot.claw.setPosition(Claw.OPEN);
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
                    robot.arm.setPosition(ZERO_OFFSET,0.4);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=6;
                    break;

                case 6:
                    if(robot.arm.getPosition()<160) {
                        OKtarget=true;
                        robot.arm.setPower(0);
                        armtarget=true;
                        robot.move.lateral(RIGHT,0.8,45);
                    }
                    break;

            }

            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)<5 && OKtarget){
                robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setPower(0);
            }
            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)>=3){
                OKtarget=false;
            }
            if(!OKtarget){
                robot.arm.armTask();
            }
            telemetry.addLine("Pos: "+robot.arm.getPosition());
            telemetry.addLine("Target: "+TargetPosInDegrees);
            telemetry.addLine("State: "+stateArm);
            telemetry.update();
        }
        /*

        robot.move.lateral(LEFT,0.6,69);

        sleep(250)

        robot.move.forward(BACKWARDS,0.6,25);

         */
    }

    public void Frontstage_CenterProp_Blue(int parking,int timer){

        robot.camera.FrontCamAtag();

        robot.wheels.GetDirection(telemetry);
        telemetry.update();


        robot.claw.setPosition(Claw.CLOSED);
        sleep(500);

        robot.joint.setPosition(Joint.DOWN);

        robot.move.lateral(RIGHT,0.4,8);
        sleep(175);

        robot.move.forward(FORWARD,0.6,57);
        sleep(175);
        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);
        robot.move.forward(BACKWARDS,0.5,5.5);
        sleep(175);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(500);
        robot.joint.setPosition(Joint.UP);

        robot.move.rotate(1,0.6,90);
        sleep(250);

        robot.move.lateral(LEFT,0.3,6.5);

        int oldtag=tagID;
        double oldPower=POWER_LiMIT;

        POWER_LiMIT=0.3;

        tagID=9;



             robot.wheels.reverseDirection();

        TurnPrecision();

             robot.wheels.setDirection();



        telemetry.addLine("Tag ID: "+tagID);


        robot.camera.openBackCam();

        tagID=oldtag;
        POWER_LiMIT=oldPower;

        telemetry.addLine("Tag ID: "+tagID);


        Targetf=0;
        Targets=0;
        Targett=0;

        Distancef =8;
        Distances=6;
        Distancet=6;

        telemetry.addLine("Tag ID: "+tagID);

        hope=0;

        sleep(250);

        robot.move.forward(BACKWARDS,0.7,150);


        robot.wheels.reverseDirection();

        Go_to_April();

        robot.wheels.setDirection();



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

                    robot.arm.setPosition(ZERO_OFFSET,0.4);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=1;
                    break;

                case 1:
                    robot.arm.setPosition(240,0.5);
                    TargetPosInDegrees=230;
                    stateArm=2;
                    break;

                case 2:
                    if(robot.arm.isOnTarget(6)) {
                        stateArm=3;
                        timerr.reset();
                    }
                    break;

                case 3:

                    if(timerr.milliseconds()>250){
                        robot.claw.setPosition(Claw.OPEN);
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
                    robot.arm.setPosition(ZERO_OFFSET,0.4);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=6;
                    break;

                case 6:
                    if(robot.arm.getPosition()<175) {
                        OKtarget=true;
                        robot.arm.setPower(0);
                        armtarget=true;
                        robot.move.lateral(LEFT,0.8,60);
                    }
                    break;

            }

            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)<5 && OKtarget){
                robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setPower(0);
            }
            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)>=3){
                OKtarget=false;
            }
            if(!OKtarget){
                robot.arm.armTask();
            }
            telemetry.addLine("Pos: "+robot.arm.getPosition());
            telemetry.addLine("Target: "+TargetPosInDegrees);
            telemetry.addLine("State: "+stateArm);
            telemetry.update();
        }





        /*
        robot.move.lateral(LEFT,0.6,58);
        sleep(250);
        robot.move.forward(BACKWARDS,0.6,25);

         */

    }
    public void Backstage_RightProp_Red(int parking, double timer){

        robot.camera.FrontCamAtag();

        robot.wheels.GetDirection(telemetry);
        telemetry.update();

        robot.claw.setPosition(Claw.CLOSED);
        sleep(500);

        robot.joint.setPosition(Joint.DOWN);




        robot.move.lateral(RIGHT,0.5,24);
        sleep(175);

        robot.move.forward(FORWARD,0.5,36.5);
        sleep(175);
        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(250);
        robot.move.forward(BACKWARDS,0.3,5.5);
        sleep(175);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(250);
        robot.joint.setPosition(0.39);

        robot.move.rotate(1,0.4,90);
        sleep(250);

        robot.move.forward(BACKWARDS,0.3,22);


        int oldtag=tagID;
        double oldPower=POWER_LiMIT;

        POWER_LiMIT=0.3;

        tagID=10;



        robot.wheels.reverseDirection();

        TurnPrecision();

        robot.wheels.setDirection();
        sleep(250);



        robot.camera.openBackCam();

        robot.move.lateral(LEFT,0.6,95);
        sleep(250);

        robot.move.forward(BACKWARDS,0.7,169);
        sleep(175);

       robot.move.rotate(-1,0.4,20);
     //   robot.move.lateral(RIGHT,0.85,50);

        tagID=oldtag;
        POWER_LiMIT=oldPower;

        telemetry.addLine("Tag ID: "+tagID);
        telemetry.update();

        Targetf=0;
        Targets=0;
        Targett=0;

        Distancef =2;
        Distances=6;
        Distancet=6;

        robot.joint.setPosition(Joint.UP);


        telemetry.addLine("Tag ID: "+tagID);


        hope=0;

       // robot.wheels.reverseDirection();

        Go_to_April();

      //  robot.wheels.setDirection();

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

                    robot.arm.setPosition(ZERO_OFFSET,0.4);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=1;
                    break;

                case 1:
                    robot.arm.setPosition(240,0.5);
                    TargetPosInDegrees=230;
                    stateArm=2;
                    break;

                case 2:
                    if(robot.arm.isOnTarget(6)) {
                        stateArm=3;
                        timerr.reset();
                    }
                    break;

                case 3:

                    if(timerr.milliseconds()>250){
                        robot.claw.setPosition(Claw.OPEN);
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
                    robot.arm.setPosition(ZERO_OFFSET,0.4);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=6;
                    break;

                case 6:
                    if(robot.arm.getPosition()<175) {
                        OKtarget=true;
                        robot.arm.setPower(0);
                        armtarget=true;
                        robot.move.lateral(LEFT,0.8,45);
                    }
                    break;

            }

            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)<5 && OKtarget){
                robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setPower(0);
            }
            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)>=3){
                OKtarget=false;
            }
            if(!OKtarget){
                robot.arm.armTask();
            }
            telemetry.addLine("Pos: "+robot.arm.getPosition());
            telemetry.addLine("Target: "+TargetPosInDegrees);
            telemetry.addLine("State: "+stateArm);
            telemetry.update();
        }

        /*

        robot.move.lateral(LEFT,0.6,69);
        sleep(250);
        robot.move.forward(BACKWARDS,0.6,25);

         */
    }


    private double ForwardPID(){


        Targetf=robot.move.returnRangeError(tagID,robot,robot.camera.atag);

        double power;


        if(Targetf==0){

            power=0;

        }else{
            double pid=forward.calculate(Targetf, Distancef);

            power = pid;

        }

        return  power;


        //  robot.wheels.setPower(-power,-power,-power,-power);

    }

    private double StrafePID(){

        Targets=robot.move.returnYawError(tagID,robot,robot.camera.atag);

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

        Targett=robot.move.returnHeadingError(tagID,robot,robot.camera.atag);

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

    public void TurnCalibration(){

        double powerTurn=StrafePID();

        double powerFrontLeft=-powerTurn;
        double powerFrontRight=powerTurn;
        double powerBackLeft=-powerTurn;
        double powerBackRight=powerTurn;

        double maxPower=mate.MaxPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);

        if(maxPower>1){

            powerFrontLeft/=maxPower;
            powerFrontRight/=maxPower;
            powerBackLeft/=maxPower;
            powerBackRight/=maxPower;
        }

        maxPower=mate.MaxPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);

        if(maxPower>POWER_LiMIT){

            double coeficient= maxPower/POWER_LiMIT;

            powerFrontLeft/=coeficient;
            powerFrontRight/=coeficient;
            powerBackLeft/=coeficient;
            powerBackRight/=coeficient;
        }


        robot.wheels.setPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);


    }

    public void TurnPrecision(){


        boolean done=false;

        while(!done){

            AprilTagDetection detection=robot.move.returnAprilTAg(tagID,robot,robot.camera.atag);

            if(detection!=null){

                telemetry.addLine("Bearing: "+detection.ftcPose.yaw);

                if(detection.ftcPose.yaw>=Distancet-3.5 && detection.ftcPose.yaw<=Distancet+3.5){

                    hope=1;
                    done=true;
                    telemetry.addLine("DONE");
                    robot.wheels.setPower(0,0,0,0);

                }else if(hope==0){
                    TurnCalibration();
                    telemetry.addLine("NOT DONE");

                }

                telemetry.update();

                /*
                telemetry.addLine("Range: "+detection.ftcPose.range);
                telemetry.addLine("Bearing: "+detection.ftcPose.bearing);
                telemetry.addLine("Yaw: "+detection.ftcPose.yaw);


                 */

                //  telemetry.update();

            }else{

                robot.wheels.setPower(0,0,0,0);

            }

        }



    }

    public void AprilPID(){

        double powerForward=ForwardPID();
        double powerStrafe=StrafePID();
        double powerTurn=TurnPID();

        double powerFrontLeft=-powerForward-powerStrafe-powerTurn;
        double powerFrontRight=-powerForward+powerStrafe+powerTurn;
        double powerBackLeft=-powerForward+powerStrafe-powerTurn;
        double powerBackRight=-powerForward-powerStrafe+powerTurn;

        double maxPower=mate.MaxPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);

        if(maxPower>1){

            powerFrontLeft/=maxPower;
            powerFrontRight/=maxPower;
            powerBackLeft/=maxPower;
            powerBackRight/=maxPower;
        }

        maxPower=mate.MaxPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);

        if(maxPower>POWER_LiMIT){

            double coeficient= maxPower/POWER_LiMIT;

            powerFrontLeft/=coeficient;
            powerFrontRight/=coeficient;
            powerBackLeft/=coeficient;
            powerBackRight/=coeficient;
        }


        robot.wheels.setPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);


    }

    public void Go_to_April(){

        ElapsedTime emergency=new ElapsedTime();
        emergency.reset();

        boolean alarm=false;

        boolean done=false;

        while(!done){

            AprilTagDetection detection=robot.move.returnAprilTAg(tagID,robot,robot.camera.atag);

            if(detection!=null){

                emergency.reset();

                telemetry.addLine("Tag ID:"+tagID);
                telemetry.addLine("Range:"+ detection.ftcPose.range);
                telemetry.addLine("YAw:"+ detection.ftcPose.yaw);
                telemetry.addLine("Bearing:"+ detection.ftcPose.bearing);



                if(detection.ftcPose.range>=Distancef-2 && detection.ftcPose.range<=Distancef+2 && detection.ftcPose.bearing>=Distancet-7 && detection.ftcPose.bearing<=Distancet+7 && detection.ftcPose.yaw>=Distances-7 && detection.ftcPose.yaw<=Distances+7){


                    telemetry.addLine("THERE");
                    hope=1;
                    done=true;
                    robot.wheels.setPower(0,0,0,0);

                }else if(hope==0){
                    AprilPID();
                    telemetry.addLine("Not there yet");

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

                }

                robot.wheels.setPower(0,0,0,0);


            }

            telemetry.update();


        }

        if(alarm){

            robot.move.lateral(RIGHT,0.8,45);

            while(opModeIsActive() && !isStopRequested()){


            }

        }


    }


}
