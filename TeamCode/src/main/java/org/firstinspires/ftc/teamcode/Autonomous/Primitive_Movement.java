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
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold_Backstage;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold_Backstage;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Wheels;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

///192.168. 43.1:8080/dash PENTRU DASHBOARD

@Config
@Autonomous(name="Primitive Movement", group = "Autonomie Primitiva")

public class Primitive_Movement extends LinearOpMode {

    private RedPropThreshold_Backstage redProp;
    private BluePropThreshold_Backstage blueProp;
    private VisionPortal camBack,camFront,visionPortal;
    private AprilTagProcessor apriltagProcesor;
    private int apriltagid,idTarget;


    private static final double PI = 3.14159265d;
    private static final double GO_TICKS_PER_REV = 537.7d;
    private Wheels wheels;
    private static final double  WHEEL_CIRCUMFERENCE =  PI * 3.7795275d;
    public static double POWER;

    public static double distx,disty,rot, straffingerror=1, lateralerror=1.1904761d, roterror=0.825d;
    public static int sign1=1;

    private IMU imu;
    private double HEADING;


    private boolean once=false;
    private Servo joint, claw;

    Joint jointPos;
    Claw clawPos;

    FtcDashboard dashboard;


    private WebcamName webcam1, webcam2;

    private boolean oldLeftBumper;
    private boolean oldRightBumper;

    private PIDController controller;

    public static double p=0.03, i=0, d=0.00001;
    public static double f=0.05;

    public static int target=3;

    public double val=0;

    private final double ticks_in_degrees=288/(360.0*0.36); /// gear ratio: 45/125=0.36

    private DcMotorEx elevator1, elevator2;
    public void runOpMode(){
    }

    public void InitCamera(){

        redProp=new RedPropThreshold_Backstage();

        apriltagProcesor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                //.3 .setLensIntrinsics(1731.46, 1731.46, 119.867, 12.6661)

                /*
                NU BUN 640x480 Camera logitech: 1731.46, 1731.46, 119.867, 12.6661
                640x480 Camera No-name:  2475.88, 2475.88, 249.071, 110.786
                 */
                .build();

        webcam1 = hardwareMap.get(WebcamName.class,"Camera1");
        webcam2 = hardwareMap.get(WebcamName.class,"Camera2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1,webcam2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(apriltagProcesor)
                .addProcessor(redProp)
                .addProcessor(blueProp)
                //.addProcessor(blueProp)
                .build();
        /*
        camFront = new VisionPortal.Builder()
                .addProcessor(redProp)
                .setCamera(hardwareMap.get(WebcamName.class,"Camera2"))
                .setCameraResolution(new Size(640,480))
                .build();

        camBack=new VisionPortal.Builder()
                .addProcessor(apriltagProcesor)
                .setCamera(hardwareMap.get(WebcamName.class,"Camera1"))
                .setCameraResolution(new Size(640,480))
                .build();

        camBack.setProcessorEnabled(apriltagProcesor,false);
        camBack.stopStreaming();

        sleep(500);

        while(camFront.getCameraState() != VisionPortal.CameraState.STREAMING){

        }

         */



         /* CA SA REDUCEM BLURAREA APRIL TAG-ULUI, ATUNCI CAND SE MISCA ROBOTUL

        ExposureControl exposure =myVisionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);
        /// telemetry.addData("Exposure: ",exposure.isExposureSupported());


        GainControl gain=myVisionPortal.getCameraControl(GainControl.class);
        gain.setGain(255);
        ///telemetry.addData("Min gain: ", gain.getMinGain());
        ///telemetry.addData("Max gain: ",gain.getMaxGain());


         */

    }


    private void telemetryAprilTag(){
        List<AprilTagDetection> currentDetections = apriltagProcesor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    public void initArm(){
        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());

        elevator1 = hardwareMap.get(DcMotorEx.class,"e1");
        elevator2 = hardwareMap.get(DcMotorEx.class,"e2");

        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);
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

    public void CamBack_Open(){

        visionPortal.setActiveCamera(webcam1);

        visionPortal.setProcessorEnabled(apriltagProcesor,true);
        visionPortal.setProcessorEnabled(redProp,false);


    }

    public void CamFront_Open_Red(){

        visionPortal.setActiveCamera(webcam2);

        visionPortal.setProcessorEnabled(blueProp,false);
        visionPortal.setProcessorEnabled(apriltagProcesor,false);
        visionPortal.setProcessorEnabled(redProp,true);

    }

    public void CamFront_Open_Blue(){

        visionPortal.setActiveCamera(webcam2);

        visionPortal.setProcessorEnabled(redProp,false);
        visionPortal.setProcessorEnabled(apriltagProcesor,false);
        visionPortal.setProcessorEnabled(blueProp,true);


    }

    public String Red_Prop_Pos(){

        return redProp.getPropPosition();

    }

    /**
     *
     * @param sign = -1 for left; and 1 for right
     * @param pow= the power for the motors
     * @param dist= distance to go
     */
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

    public void Strafing(int sign,double pow, double dist){

        if(sign>1){
            sign=1;
        }
        if(sign<-1){
            sign=-1;
        }

        dist=dist*straffingerror;

        dist=(int)(Inch_To_Ticks(Cm_To_Inch(dist))*(1/Math.sin(45) ) );

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        double x=dist-(int)Inch_To_Ticks(Cm_To_Inch(10*pow/0.6));

        while(Math.abs(wheels.frontLeft.getCurrentPosition())<Math.abs(x)&&
                Math.abs(wheels.frontRight.getCurrentPosition())<Math.abs(x) &&
                Math.abs(wheels.backRight.getCurrentPosition())<Math.abs(x) &&
                Math.abs(wheels.backLeft.getCurrentPosition())<Math.abs(x)){
            wheels.setPower(sign*pow,-sign*pow,-sign*pow,sign*pow);

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

            wheels.setPower(sign*pow,-sign*pow,-sign*pow,sign*pow);

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


    /**
     *
     * @param sign -1 to go backwards; and 1 to go forward
     * @param pow the power for the motors
     * @param dist the distance to go
     */

    public void GoFront(int sign, double pow, double dist){

        if(sign>1){
            sign=1;
        }
        if(sign<-1){
            sign=-1;
        }

        dist=dist*1.1363636;

        int ticks=(int)(Inch_To_Ticks(Cm_To_Inch(dist))*35/42);

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        wheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheels.setTargetPosition(ticks*sign,ticks*sign,ticks*sign,ticks*sign);
        wheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wheels.setPower(pow);

        double x=ticks-(int)Inch_To_Ticks(Cm_To_Inch(10));

        while(wheels.frontRight.isBusy() && wheels.frontLeft.isBusy() && wheels.backRight.isBusy() && wheels.backLeft.isBusy()){

            if(Math.abs(wheels.frontRight.getCurrentPosition())>Math.abs(x) &&
                    Math.abs(wheels.frontLeft.getCurrentPosition())>Math.abs(x) &&
                    Math.abs(wheels.backRight.getCurrentPosition())>Math.abs(x) &&
                    Math.abs(wheels.backLeft.getCurrentPosition())>Math.abs(x)){

                if(pow>0.01){
                    pow=pow-0.1;
                }
                if(pow<0.01){
                    pow=0.01;
                }

                wheels.setPower(sign*pow);

            }

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


    /**
     *
     * @param sign -1 rotate anticlockwise and 1 to rotate clockwise
     * @param pow the power that is gonna be used to rotate
     * @param deg how many degrees it should rotate
     */

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

    public void AllMove(double distx,double disty, double rot){

        distx=(int)(Inch_To_Ticks(Cm_To_Inch(distx)));
        disty=(int)(Inch_To_Ticks(Cm_To_Inch(disty)));
        rot=Math.toRadians(rot);

        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while(!isStopRequested() && opModeIsActive() &&
                ( (Math.abs(wheels.frontLeft.getCurrentPosition())<Math.abs(distx)&&
                        Math.abs(wheels.frontRight.getCurrentPosition())<Math.abs(distx) &&
                        Math.abs(wheels.backRight.getCurrentPosition())<Math.abs(distx) &&
                        Math.abs(wheels.backLeft.getCurrentPosition())<Math.abs(distx)) ||
                        (Math.abs(wheels.frontLeft.getCurrentPosition())<Math.abs(disty)&&
                                Math.abs(wheels.frontRight.getCurrentPosition())<Math.abs(disty) &&
                                Math.abs(wheels.backRight.getCurrentPosition())<Math.abs(disty) &&
                                Math.abs(wheels.backLeft.getCurrentPosition())<Math.abs(disty))||
                        (Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))-Math.abs(HEADING)<Math.abs(rot)))){

            double x= distx-maxim(wheels.backLeft.getCurrentPosition(),wheels.backRight.getCurrentPosition(),
                    wheels.frontLeft.getCurrentPosition(),wheels.frontRight.getCurrentPosition());
            double y=disty- maxim(wheels.backLeft.getCurrentPosition(),wheels.backRight.getCurrentPosition(),
                    wheels.frontLeft.getCurrentPosition(),wheels.frontRight.getCurrentPosition());
            double z=rot- imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            moveRobot(x,y,z);

        }

        wheels.setPower(0);
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        HEADING=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


    }

    public double maxim(double a, double b, double c, double d){

        return Math.max(Math.max(Math.max(a,b),c),d) ;

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


    public double Correcting_Yaw(){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;;

        while(!targetFound){

            List<AprilTagDetection> currentDetections = apriltagProcesor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null && (detection.id==5 || detection.id==2))){
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

        }

        return Range.clip(desiredTag.ftcPose.bearing* 1, -0.7, 0.7) ;
    }

    public void Please_Yaw(double pow){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;;
        int sign=1;

        while(!targetFound){

            List<AprilTagDetection> currentDetections = apriltagProcesor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null && (detection.id==5 || detection.id==2))){
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

        }

        if(desiredTag.ftcPose.yaw>0){

            sign=-1;

        }

        Rotate(sign,pow,desiredTag.ftcPose.yaw);
        sleep(500);

    }

    public void April_Please(double pow, int atagID){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;;
        boolean ok=false;

        double rangeError=10,headingError=10,yawError=10;
        int signforward=1,signstrafe=1,signturn=1;

        while(!targetFound){

            List<AprilTagDetection> currentDetections = apriltagProcesor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) && (detection.id==atagID)){
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

        }

        rangeError=desiredTag.ftcPose.y-5;
        headingError=desiredTag.ftcPose.yaw;
        yawError=desiredTag.ftcPose.x;

        if(rangeError>0){
            signforward=-1;
        }

        if(headingError<0){

            signturn=-1;

        }

        if(yawError>0){
            signstrafe=-1;
        }

        Forward(signforward,pow,rangeError);
        sleep(500);

        lateral(signstrafe,pow,yawError);
        sleep(500);

        Rotate(signturn,pow,headingError);
        sleep(500);

    }

    public void Go_To_April(double pow, int atagID){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;;
        boolean ok=false;

        double rangeError=10,headingError=10,yawError=10;
        int signforward=1,signstrafe=1,signturn=1;

        while(!targetFound){

            List<AprilTagDetection> currentDetections = apriltagProcesor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) && (detection.id==atagID)){
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

        }

        rangeError=desiredTag.ftcPose.range-5;
        headingError=desiredTag.ftcPose.bearing;
        yawError=desiredTag.ftcPose.yaw;

        if(rangeError>0){
            signforward=-1;
        }

        if(headingError<0){

            signturn=-1;

        }

        if(yawError>0){
            signstrafe=-1;
        }

        Forward(signforward,pow,rangeError);
        sleep(500);

        lateral(signstrafe,pow,yawError);
        sleep(500);

        Rotate(signturn,pow,headingError);
        sleep(500);

    }

    public void Move_To_April(int atagID){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;;
        boolean ok=false;

        double rangeError=10,headingError=10,yawError=10;

        while(!ok){

            List<AprilTagDetection> currentDetections = apriltagProcesor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) && (detection.id==atagID)){
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

            if(targetFound){

                rangeError      = (desiredTag.ftcPose.range - 5);
                headingError    = desiredTag.ftcPose.bearing;
                yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive  = Range.clip(rangeError * 1, -0.7, 0.7);
                double turn   = Range.clip(headingError * 1, -0.7, 0.7) ;
                double strafe = Range.clip(-yawError * 1, -0.7, 0.7);


                moveRobot(drive,strafe,turn);

            }

            if(rangeError>=-4 && rangeError<=4 && headingError>=0 && headingError<=4 && yawError>=0 && yawError<=0.){

                ok=true;

            }


        }


    }

    public double Cm_To_Inch (double cm){

        return cm*0.393700787d;

    }


    public int Inch_To_Ticks(double inch){
        return (int)((inch/WHEEL_CIRCUMFERENCE)*GO_TICKS_PER_REV);
    }

}
