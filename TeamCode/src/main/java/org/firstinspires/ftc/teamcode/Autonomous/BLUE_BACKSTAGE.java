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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Wheels;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="BLUE BACKSTAGE", group = "Red Routes")

public class BLUE_BACKSTAGE extends LinearOpMode {

    private int PARKING=-1; //-1 for left parking and 1 for right
    private WebcamName backCam, frontCam;
    private BluePropThreshold blueProp;
    private AprilTagProcessor atag;
    private VisionPortal vision;
    private int tagID=-1;
    public static double rangeErrorGAIN=0.02, headingErrorGAIN=0.01,yawErrorGAIN=0.015;

    public static double MAXrange=0.3,MAXheading=0.2,MAXyaw=0.2;


    private static final double PI = 3.14159265d;
    private static final double GO_TICKS_PER_REV = 537.7d;
    private Wheels wheels;

    private static final double  WHEEL_CIRCUMFERENCE =  PI * 3.7795275d;
    public static double POWER=0.6;

    public static double lateralerror=1.1904761d, roterror=0.85672725d;

    private IMU imu;
    private double HEADING;


    private Servo joint, claw;

    Joint jointPos;
    Claw clawPos;

    FtcDashboard dashboard;


    private PIDController controller;

    public static double p=0.03, i=0, d=0.00001;
    public static double f=0.05;

    public static int target=3;

    public double val=0;

    private final double ticks_in_degrees=288/(360.0*0.36); /// gear ratio: 45/125=0.36

    private DcMotorEx elevator1, elevator2;

    @Override
    public void runOpMode() {

        Init_Cameras();
        InitIMU();
        InitMotors();
        InitArm();

        jointPos=new Joint();
        joint=hardwareMap.get(Servo.class,"joint");

        clawPos=new Claw();
        claw= hardwareMap.get(Servo.class,"claw");

        Open_FrontCam();

        String propPosition=blueProp.getPropPosition();

        boolean once=true;


        waitForStart();

        claw.setPosition(Claw.INTERMEDIARY);

        sleep(1500);

        joint.setPosition(Joint.DOWN);

        sleep(1500);

        claw.setPosition(Claw.CLOSED);

        sleep(1500);

        lateral(-1,0.6,95);

        /*
        while ((propPosition.equals("nope") || once) && opModeIsActive() && !isStopRequested()){

            telemetry.addLine("Nope :( "+propPosition);
            propPosition=blueProp.getPropPosition();

            if(propPosition.equals("left")){

                telemetry.addLine(propPosition);
                telemetry.update();
                tagID=4;

                Backstage_LeftProp_Blue(PARKING,0);

            }else if(propPosition.equals("center")){

                telemetry.addLine(propPosition);
                telemetry.update();
                tagID=5;

                Backstage_CenterProp_Blue(PARKING,0);

            }else if(propPosition.equals("right")){

                telemetry.addLine(propPosition);
                telemetry.update();
                tagID=6;

                Backstage_RightProp_Blue(PARKING,0);

            }

            telemetry.update();

        }

         */


    }

    public void Backstage_LeftProp_Blue(int parking,int timer){

        Open_BackCam();

        waitForStart();

        claw.setPosition(Claw.INTERMEDIARY);
        sleep(1000);

        joint.setPosition(Joint.DOWN);
        sleep(1000);

        claw.setPosition(Claw.CLOSED);
        sleep(1000);

        Forward(1,0.7,64);
        sleep(1000);


        Rotate(-1,0.5,90);
        sleep(1000);

        claw.setPosition(Claw.INTERMEDIARY);
        sleep(1000);

        joint.setPosition(Joint.UP);
        sleep(1000);

        lateral(-1,0.7,61);
        sleep(1000);

        Forward(1,0.7,45);

        /*
        Forward(1,0.5,7);
        sleep(500);

        claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);

        Forward(-1,0.4,3);
        sleep(500);

        claw.setPosition(Claw.CLOSED);
        sleep(100);

        joint.setPosition(Joint.UP);

        //ove_to_April(tagID);

        moveArm(200);
        sleep(500);

        claw.setPosition(Claw.OPEN);
        sleep(500);

        moveArm(0);

        if(parking=="LEFT"){

            lateral(-1,0,0);

        }else if(parking=="RIGHT"){

            lateral(1,0,0);

        }


         */

       // lateral(parking,0.6,20);//DOAR PARCARE




    }

    public void Backstage_CenterProp_Blue(int parking,int timer){

        Open_BackCam();

        waitForStart();

        claw.setPosition(clawPos.INTERMEDIARY);
        sleep(1000);
        joint.setPosition(jointPos.DOWN);
        sleep(1000);

        claw.setPosition(Claw.CLOSED);
        sleep(1000);



        sleep(1000);
        Forward(1,0.7,66);
        sleep(1000);


        claw.setPosition(clawPos.INTERMEDIARY);
        sleep(1000);

        joint.setPosition(Joint.UP);

       // Forward(-1,0.5,4);
        //sleep(1000);

       // claw.setPosition(clawPos.CLOSED);
       // sleep(1000);

        Forward(-1,0.7,65);
        sleep(1000);

        lateral(-1,0.5,45);
        sleep(1000);

       // claw.setPosition(Claw.INTERMEDIARY);
       // sleep(1000);

        //joint.setPosition(Joint.UP);

/*
        Rotate(1,0,90);
        sleep(500);

        Move_To_April(tagID);
        sleep(500);

        moveArm(200);

        sleep(500);

        setPosition(clawPos.OPEN);

        moveArm(0);

        sleep(500);


        if(parking=="LEFT"){

            lateral(-1,0,0);

        }else if(parking=="RIGHT"){

            lateral(1,0,0);

        }

       */

        lateral(parking,0.5,20);//PARCAM SI ATAT


    }

    public void Backstage_RightProp_Blue(int parking, double timer){

        Open_BackCam();

        waitForStart();


        claw.setPosition(Claw.INTERMEDIARY);
        sleep(1000);

        joint.setPosition(Joint.DOWN);
        sleep(1000);

        claw.setPosition(Claw.CLOSED);
        sleep(1000);

        Forward(1,0.7,64);
        sleep(1000);


        Rotate(1,0.5,90);
        sleep(1000);

        claw.setPosition(Claw.INTERMEDIARY);
        sleep(1000);

        joint.setPosition(Joint.UP);
        sleep(1000);

        lateral(1,0.7,61);
        sleep(1000);

        Forward(-1,0.7,45);

        /*
        Open_BackCam();

        waitForStart();

        joint.setPosition(Joint.UP);
        sleep(500);

        claw.setPosition(Claw.CLOSED);
        sleep(500);

        Forward(1,0.7,61);
        sleep(500);

        Rotate(1,0.5,90);
        sleep(500);

        joint.setPosition(Joint.DOWN);

        Forward(-1,0.6,25);
        sleep(500);

        claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);

        Forward(-1,0.4,3.5);
        sleep(500);

        claw.setPosition(Claw.CLOSED);
        sleep(100);

        joint.setPosition(Joint.UP);

         */

        /*
        Move_to_April(tagID);

        moveArm(200);
        sleep(500);

        claw.setPosition(Claw.OPEN);
        sleep(500);

        moveArm(0);
        sleep(500);

        if(parking=="LEFT"){

            lateral(-1,0,0);

        }else if(parking=="RIGHT"){

            lateral(1,0,0);

        }

         */

        lateral(parking,0.6,30);



    }


    public void Init_Cameras(){ ///PORNESTE CU BACKCAM-UL ACTIVAT

        atag= new AprilTagProcessor.Builder().build();

        blueProp=new BluePropThreshold();

        backCam = hardwareMap.get(WebcamName.class, "Camera1");
        frontCam = hardwareMap.get(WebcamName.class, "Camera2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(backCam, frontCam);

        vision = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(atag,blueProp)
                .build();

        while(vision.getCameraState()!= VisionPortal.CameraState.STREAMING){

        }

        vision.setProcessorEnabled(blueProp,false);
    }

    public void Open_BackCam(){

        if (vision.getCameraState() == VisionPortal.CameraState.STREAMING) {
            vision.setActiveCamera(backCam);
        }

        vision.setProcessorEnabled(atag,true);
        vision.setProcessorEnabled(blueProp,false);

        while(vision.getCameraState()!= VisionPortal.CameraState.STREAMING){

        }
    }

    public void Open_FrontCam(){

        if (vision.getCameraState() == VisionPortal.CameraState.STREAMING) {
            vision.setActiveCamera(frontCam);
        }

        vision.setProcessorEnabled(atag,false);
        vision.setProcessorEnabled(blueProp,true);

        while(vision.getCameraState()!= VisionPortal.CameraState.STREAMING){

        }
    }



    public void InitArm(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

        elevator1 = hardwareMap.get(DcMotorEx.class,"e1");
        elevator2 = hardwareMap.get(DcMotorEx.class,"e2");

        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveArm(int desiredTarget){//TODO: RESCRIS

        if(elevator1.getCurrentPosition()<desiredTarget){

            for(target = elevator1.getCurrentPosition();target<=desiredTarget;target++){
                controller.setPID(p, i, d);
                int elepos = elevator1.getCurrentPosition();
                double pid = controller.calculate(elepos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees));

                double power = pid + ff;

                elevator1.setPower(power);
                elevator2.setPower(power);
                sleep(250);
            }

        }else if(elevator1.getCurrentPosition()>desiredTarget){

            for(target = elevator1.getCurrentPosition();target>=0;target--){
                controller.setPID(p, i, d);
                int elepos = elevator1.getCurrentPosition();
                double pid = controller.calculate(elepos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees));

                double power = pid + ff;

                elevator1.setPower(power);
                elevator2.setPower(power);
                sleep(250);
            }

        }


    }

    public void InitMotors(){
        wheels = new Wheels(hardwareMap);
        wheels.setDirection();
        wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void InitIMU(){

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        imu.resetYaw();

        HEADING=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

    }


    public void Forward(int sign,double pow, double dist){

        if(sign>1){
            sign=1;
        }
        if(sign<-1){
            sign=-1;
        }

        dist=(int)(Inch_To_Ticks(Cm_To_Inch(dist)));

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double x=dist-(int)Inch_To_Ticks(Cm_To_Inch(10*pow/0.6));

        while(Math.abs(wheels.frontLeft.getCurrentPosition())<Math.abs(x)&&
                Math.abs(wheels.frontRight.getCurrentPosition())<Math.abs(x) &&
                Math.abs(wheels.backRight.getCurrentPosition())<Math.abs(x) &&
                Math.abs(wheels.backLeft.getCurrentPosition())<Math.abs(x)){

            wheels.setPower(sign*pow);

        }

        while(Math.abs(wheels.frontLeft.getCurrentPosition())<Math.abs(dist)&&
                Math.abs(wheels.frontRight.getCurrentPosition())<Math.abs(dist) &&
                Math.abs(wheels.backRight.getCurrentPosition())<Math.abs(dist) &&
                Math.abs(wheels.backLeft.getCurrentPosition())<Math.abs(dist)){

            if(pow>0.01){
                pow=pow-0.4;
            }
            if(pow<0.01){
                pow=0.01;
            }

            wheels.setPower(sign*pow);

        }

        wheels.setPower(0);
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }
    public void lateral (int sign,double pow, double dist){

        if(sign>1){
            sign=1;
        }
        if(sign<-1){
            sign=-1;
        }

        dist=dist*lateralerror;

        int ticks=(int)((Inch_To_Ticks(Cm_To_Inch(dist))*(1/Math.sin(45) ) )*35/42);

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheels.setTargetPosition(ticks*sign,-ticks*sign, -ticks*sign, ticks*sign);
        wheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheels.setPower(pow);

        wheels.waitMotors();

        wheels.setPower(0);

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Rotate(int sign, double pow, double deg){

        if(sign>1){
            sign=1;
        }
        if(sign<-1) {
            sign = -1;
        }

        deg=deg*roterror;

        double botHeading=Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))-Math.abs(HEADING);
        deg=Math.toRadians(deg);

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while(Math.abs(botHeading)<Math.abs(deg)){

            wheels.setPower(pow*sign, -pow*sign, pow*sign,-pow*sign);

            botHeading = Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))-Math.abs(HEADING);

            telemetry.addLine("Heading"+ imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        wheels.setPower(0);
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        HEADING=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        telemetry.addLine("Heading"+ imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();

    }

    public void Move_to_April(int idtag){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;
        ;
        boolean ok = false;
        int oldtarget=-1;

        double rangeError = 0, headingError = 0, yawError = 0;

        while (!ok && opModeIsActive() && !isStopRequested()) {

            List<AprilTagDetection> currentDetections = atag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) && (detection.id == 1)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

            if (targetFound) {

                rangeError = (desiredTag.ftcPose.range - 1);
                headingError = desiredTag.ftcPose.bearing;
                yawError = desiredTag.ftcPose.yaw; //1.7795

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive = Range.clip(rangeError * rangeErrorGAIN, -MAXrange,MAXrange );
                double turn = Range.clip(headingError * headingErrorGAIN, -MAXheading, MAXheading);
                double strafe = Range.clip(-yawError * yawErrorGAIN, -MAXyaw, MAXyaw);


                //moveRobot(-drive, -strafe, turn);

                moveRobot(-drive, -strafe, turn);


            }else{

                moveRobot(0,0,0);

            }

            if (rangeError < 7) {

                ok = true;
                moveRobot(0,0,0);

            }


        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x -y -yaw;
        double frontRightPower   =  x +y +yaw;
        double backLeftPower     =  x +y -yaw;
        double backRightPower    =  x -y +yaw;

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

        wheels.setPower(frontLeftPower,frontRightPower,backLeftPower,backRightPower);
    }


    public double Cm_To_Inch (double cm){

        return cm*0.393700787d;

    }


    public int Inch_To_Ticks(double inch){
        return (int)((inch/WHEEL_CIRCUMFERENCE)*GO_TICKS_PER_REV);
    }

    public double Ticks_To_Inch(double ticks){
        return (ticks/GO_TICKS_PER_REV)*WHEEL_CIRCUMFERENCE;
    }

}
