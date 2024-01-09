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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold;
//import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold_Backstage;
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

@Autonomous(name="April Basic", group = "Robot")

public class April_Basic extends LinearOpMode {

    private WebcamName webcam1, webcam2;

   // private RedPropThreshold_Backstage redProp;

   // private BluePropThreshold blueProp;

    private AprilTagProcessor atag;


    private VisionPortal visionPortal;


    private static final double PI = 3.14159265d;
    private static final double GO_TICKS_PER_REV = 537.7d;
    private Wheels wheels;
    public static double distx,disty,rot, straffingerror=1, lateralerror=1.1904761d, roterror=0.825d;

    private static final double  WHEEL_CIRCUMFERENCE =  PI * 3.7795275d;
    public static double POWER;

    public static double rangeErrorGAIN=0.02, headingErrorGAIN=0.01,yawErrorGAIN=0.015;

    public static double MAXrange=0.3,MAXheading=0.2,MAXyaw=0.2;


    private IMU imu;
    private double HEADING;

    @Override
    public void runOpMode() {

        Init_Cameras();
        InitMotors();
        InitIMU();

        waitForStart();

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;
        ;
        boolean ok = false;
        int oldtarget=-1;

        double xpos = 0, ypos= 0, yawpos = 0;

        while(targetFound==false){

            List<AprilTagDetection> currentDetections = atag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) && (detection.id == 5)) {
                    targetFound = true;
                    desiredTag = detection;
                    xpos=desiredTag.ftcPose.x;
                    ypos=desiredTag.ftcPose.y;
                    yawpos=desiredTag.ftcPose.yaw;
                    break;  // don't look any further.
                }
            }

        }

        Rotate(11,0.7,yawpos);
        sleep(3000);


        Forward(-1,0.7,ypos);
        sleep(3000);


        lateral(-1,0.7,xpos);

    }

    public void Init_Cameras(){

        atag= new AprilTagProcessor.Builder().build();

     //   redProp=new RedPropThreshold_Backstage();
     //   blueProp=new BluePropThreshold();

        webcam1 = hardwareMap.get(WebcamName.class, "Camera1");
        webcam2 = hardwareMap.get(WebcamName.class, "Camera2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
              //  .addProcessors(atag,redProp,blueProp)
                .build();

        while(visionPortal.getCameraState()!= VisionPortal.CameraState.STREAMING){

        }

       // visionPortal.setProcessorEnabled(redProp,false);
       // visionPortal.setProcessorEnabled(blueProp,false);

    }

    public void Open_Cam1(){

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.setActiveCamera(webcam1);
        }

        visionPortal.setProcessorEnabled(atag,true);
      //  visionPortal.setProcessorEnabled(redProp,false);
      //  visionPortal.setProcessorEnabled(blueProp,false);

    }

    public void Open_Cam2(){

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.setActiveCamera(webcam2);
        }

        visionPortal.setProcessorEnabled(atag,false);
      //  visionPortal.setProcessorEnabled(redProp,true);
      //  visionPortal.setProcessorEnabled(blueProp,true);

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

        sleep(1000);

        if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)<HEADING){
            Rotate(-1,POWER,-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+ HEADING);
        }else if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)>HEADING){
            Rotate(1,POWER,imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-HEADING);
        }

        HEADING=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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

        sleep(1000);

        if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)<HEADING){
            Rotate(-1,POWER,-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+ HEADING);
        }else if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)>HEADING){
            Rotate(1,POWER,imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-HEADING);
        }

        HEADING=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void Rotate(int sign, double pow, double deg){

        if(sign>1){
            sign=1;
        }
        if(sign<-1){
            sign=-1;
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

    public double Cm_To_Inch (double cm){

        return cm*0.393700787d;

    }

    public double Inch_To_Cm(double inch){
        return inch/0.393700787d;
    }

    public int Inch_To_Ticks(double inch){
        return (int)((inch/WHEEL_CIRCUMFERENCE)*GO_TICKS_PER_REV);
    }

}
