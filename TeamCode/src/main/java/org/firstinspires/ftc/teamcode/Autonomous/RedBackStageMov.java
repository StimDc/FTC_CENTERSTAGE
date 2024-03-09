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


import android.os.Environment;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;
import org.firstinspires.ftc.teamcode.Implementations.Math.MathFunc;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.concurrent.TimeUnit;


public class RedBackStageMov {

    private int PARKING = 1; //-1 for left parking and 1 for right

    public static double target;
    private Robot robot;

    private int tagID;

    private boolean FALSE = false;

    private final double ZERO_OFFSET = 70.0 - 3.85;
    private double TargetPosInDegrees = 70.0 - 3.85;
    private Telemetry telemetry;

    private ElapsedTime AUTO;


    public MathFunc mate;


    public FtcDashboard dashboard;
    private PIDController forward, strafe, turn;

    public static double Pf = 0.02d, If = 0d, Df = 0d;
    public static double Ps = 0.045d, Is = 0d, Ds = 0d;
    public static double Pt = 0.02d, It = 0.01d, Dt = 0.00005d;


    private static double Targetf = 0, Targets = 0, Targett;

    public double val = 0;


    public static double Distancef = 7, Distances = 6, Distancet = 6;

    public static double POWER_LiMIT = 0.4;

    private int hope = 0;

    public RedBackStageMov(Robot robot, Telemetry telemetry, int tagID) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.tagID = tagID;
    }

    public void passTag(int tagID) {
        this.tagID = tagID;
    }

    public void init() {
        // this.robot.camera.openFrontCam();
        target = this.robot.arm.ZERO_OFFSET;

        AUTO = new ElapsedTime();

        dashboard = FtcDashboard.getInstance();

        forward = new PIDController(Pf, If, Df);
        forward.setPID(Pf, If, Df);

        strafe = new PIDController(Ps, Is, Ds);
        strafe.setPID(Ps, Is, Ds);

        turn = new PIDController(Pt, It, Dt);
        turn.setPID(Pt, It, Dt);

        this.telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());


        //  String propPosition=this.robot.camera.GetPropPosition();

        //  this.telemetry.addLine("Prop: "+propPosition);
        //  this.telemetry.update();

        boolean once = true;

        this.robot.wheels.GetDirection(this.telemetry);
        this.telemetry.update();


    }

    public void backStageLeftProp(int parking, int timer) throws IOException {

        int parkDist;

        if (parking == -1) {

            parkDist = 70;

        } else {

            parkDist = 40;

        }

        this.AUTO.reset();

        POWER_LiMIT = 0.35;

        this.robot.camera.openBackCam();

        setManualExposure(1, 1);


        this.robot.wheels.GetDirection(this.telemetry);
        this.telemetry.update();

        robot.claw.setPosition(Claw.CLOSED);
        sleep(750);
        robot.joint.setPosition(Joint.DOWN);
        sleep(900);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(900);

        this.robot.wheels.GetDirection(telemetry);
        telemetry.update();

        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(500);

        this.robot.joint.setPosition(Joint.DOWN);


        this.robot.move.forward(FORWARD, 0.6, 45);
        sleep(175);

        this.robot.move.rotate(-1, 0.6, 51.5);
        sleep(200);


        this.robot.move.forward(FORWARD, 0.4, 12);
        sleep(200);

        this.robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);


        this.robot.move.forward(BACKWARDS, 0.5, 5);
        sleep(175);
        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(500);
        this.robot.joint.setPosition(Joint.UP);
        sleep(1500);

        this.robot.move.rotate(-1, 0.6, 90-51.5);
        sleep(200);

        this.robot.move.forward(BACKWARDS, 0.5, 30);
        sleep(500);

        this.robot.move.lateral(LEFT,0.4,20);

        this.robot.claw.setPosition(Claw.OPEN);






    }

    public void backStageCenterPropBackup(int parking, int timer) throws IOException{
        int parkDist;

        if(parking==-1){

            parkDist=60;

        }else{

            parkDist=60;

        }

        this.AUTO.reset();


        this.robot.camera.openBackCam();

        setManualExposure(1,1);


        this.robot.wheels.GetDirection(this.telemetry);
        this.telemetry.update();

        robot.claw.setPosition(Claw.CLOSED);
        sleep(900);
        robot.joint.setPosition(Joint.DOWN);
        sleep(900);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(900);


        this.robot.move.lateral(RIGHT,0.4,8);
        sleep(175);

        this.robot.move.forward(FORWARD,0.6,60);//57
        sleep(175);
        this.robot.joint.setPosition(Joint.DOWN);
        sleep(500);
        this.robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);
        this.robot.move.forward(BACKWARDS,0.5,5.5);
        sleep(175);
        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(500);
        this.robot.joint.setPosition(Joint.UP);

        this.robot.move.forward(BACKWARDS,0.6, 50);
        this.robot.move.lateral(RIGHT, 0.6,50);
    }
    public void backStageCenterProp(int parking,int timer) throws IOException {

        int parkDist;

        if(parking==-1){

            parkDist=60;

        }else{

            parkDist=60;

        }

        this.AUTO.reset();


        this.robot.camera.openBackCam();

        setManualExposure(1,1);


        this.robot.wheels.GetDirection(this.telemetry);
        this.telemetry.update();

        robot.claw.setPosition(Claw.CLOSED);
        sleep(900);
        robot.joint.setPosition(Joint.DOWN);
        sleep(900);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(900);


        this.robot.move.lateral(RIGHT,0.4,8);
        sleep(175);

        this.robot.move.forward(FORWARD,0.6,60);//57
        sleep(175);
        this.robot.joint.setPosition(Joint.DOWN);
        sleep(500);
        this.robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);
        this.robot.move.forward(BACKWARDS,0.5,5.5);
        sleep(175);
        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(500);
        this.robot.joint.setPosition(Joint.UP);

        this.robot.move.forward(BACKWARDS,0.5,60-3.5);
        sleep(500);

        this.robot.move.lateral(RIGHT,0.4,50);
        this.robot.claw.setPosition(Claw.OPEN);
    }
    public void backStageRightPropBackup(int parking,double timer) throws IOException{
        int parkDist;

        if(parking==-1){

            parkDist=40;

        }else{

            parkDist=70;

        }

        this.robot.camera.openBackCam();

        setManualExposure(1,1);


        this.robot.wheels.GetDirection(this.telemetry);
        this.telemetry.update();


        robot.claw.setPosition(Claw.CLOSED);
        sleep(750);
        robot.joint.setPosition(Joint.DOWN);
        sleep(900);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(900);


        this.robot.wheels.GetDirection(this.telemetry);
        this.telemetry.update();

        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(500);

        this.robot.joint.setPosition(Joint.DOWN);



        this.robot.move.lateral(RIGHT,0.4,24);
        sleep(175);

        this.robot.move.forward(FORWARD,0.6,40);
        sleep(175);
        this.robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(250);
        this.robot.move.forward(BACKWARDS,0.5,5.5);
        sleep(175);
        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(250);
        this.robot.joint.setPosition(Joint.UP);

        this.robot.move.forward(BACKWARDS,0.6,50-7.5);
        this.robot.move.forward(RIGHT,0.6,50);
    }
    public void backStageRightProp(int parking, double timer) throws IOException {

        int parkDist;

        if(parking==-1){

            parkDist=40;

        }else{

            parkDist=70;

        }

        this.robot.camera.openBackCam();

        setManualExposure(1,1);


        this.robot.wheels.GetDirection(this.telemetry);
        this.telemetry.update();


        robot.claw.setPosition(Claw.CLOSED);
        sleep(750);
        robot.joint.setPosition(Joint.DOWN);
        sleep(900);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(900);


        this.robot.wheels.GetDirection(this.telemetry);
        this.telemetry.update();

        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(500);

        this.robot.joint.setPosition(Joint.DOWN);



        this.robot.move.lateral(RIGHT,0.4,24);
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

        this.robot.move.forward(BACKWARDS,0.6,36.5-7.5);
        this.robot.move.forward(RIGHT,0.6,50);




    }

    private double ForwardPID(){


        Targetf=this.robot.move.returnRangeError(this.tagID,this.robot,this.robot.camera.atag);

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


        this.robot.wheels.setPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);


    }

    public void Go_to_April() throws IOException {

        ElapsedTime emergency=new ElapsedTime();
        emergency.reset();

        boolean alarm=false;

        boolean done=false;
        boolean exit = false;

        while(!done){

            AprilTagDetection detection=this.robot.move.returnAprilTAg(this.tagID,this.robot,this.robot.camera.atag);

            if(detection!=null){

                emergency.reset();

                if(detection.ftcPose.range>=Distancef-2 && detection.ftcPose.range<=Distancef+2 && detection.ftcPose.bearing>=Distancet-7 && detection.ftcPose.bearing<=Distancet+7 && detection.ftcPose.yaw>=Distances-7 && detection.ftcPose.yaw<=Distances+7){

                    hope=1;
                    done=true;
                    this.robot.wheels.setPower(0,0,0,0);

                }else if(hope==0){
                    AprilPID();

                }


                telemetry.addLine("Range: "+detection.ftcPose.range);
                telemetry.addLine("Bearing: "+detection.ftcPose.bearing);
                telemetry.addLine("Yaw: "+detection.ftcPose.yaw);




                telemetry.update();
                String logFilePath = String.format("%s/FIRST/data/debug.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
                FileWriter fw = new FileWriter(logFilePath);
                fw.write(detection.ftcPose.toString());

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

            FALSE=true;

        }


    }



    private boolean   setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (this.robot.camera.vision == null) {
            return false;
        }

        // Wait for the camera to be open
        if (this.robot.camera.vision.getCameraState() != VisionPortal.CameraState.STREAMING) {
            this.telemetry.addData("Camera", "Waiting");
            this.telemetry.update();
            while ((this.robot.camera.vision.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            this.telemetry.addData("Camera", "Ready");
            this.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!false)
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = this.robot.camera.vision.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(7, TimeUnit.MILLISECONDS);
            sleep(20);

            this.telemetry.addLine("Exposure Ceva: "+exposureControl.getExposure(TimeUnit.MILLISECONDS));

            // Set Gain.
            GainControl gainControl = this.robot.camera.vision.getCameraControl(GainControl.class);

            this.telemetry.addLine("Gain Ceva: "+gainControl);


            if(gainControl!=null){
                boolean haide= gainControl.setGain(255);
                this.telemetry.addLine("Gain: "+gainControl.getGain());

            }
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }



}
