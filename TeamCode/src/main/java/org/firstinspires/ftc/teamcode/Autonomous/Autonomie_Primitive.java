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

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Wheels;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

///192.168. 43.1:8080/dash PENTRU DASHBOARD

@Config
@Autonomous(name="Autonomie Primitiva", group = "Autonomie Primitiva")

public class Autonomie_Primitive extends LinearOpMode {

    private RedPropThreshold redProp;
    private VisionPortal visionPortal;
    private AprilTagProcessor apriltagProcesor;
    private int apriltagid;


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

    FtcDashboard dashboard;

    public static int mode=0;




    @Override
    public void runOpMode() {

       // InitCamera();
        InitMotors();
        InitIMU();

        joint=hardwareMap.get(Servo.class,"joint");

        telemetry.addLine("Heading"+ imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));


        dashboard=FtcDashboard.getInstance();




        waitForStart();

        if(!once){
            joint.setPosition(Joint.UP);
            once=true;
        }

        while(!isStopRequested() && opModeIsActive()){
            telemetry.addData("Rotation: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            switch(mode){
                case 1:
                    GoFront(sign1,POWER,distx);
                    break;
                case 2:
                    lateral(sign1,POWER,disty);
                    break;
                case 3:
                    Rotate(sign1,POWER,rot);
                    break;
                case 4:
                    Forward(sign1,POWER,distx);
                    break;
                case 5:
                    Strafing(sign1,POWER,disty);
                    break;
                case 6:
                    AllMove(distx,disty,rot);
                    
            }
            sleep(1000);
            mode=0;

            if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)<HEADING){
                Rotate(-1,POWER,-Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))+ Math.abs(HEADING));
            }else if(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)>HEADING){
                Rotate(1,POWER,Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))-Math.abs(HEADING));
            }

            HEADING=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            sleep(1000);

        }



    }

    public void InitCamera(){

        redProp=new RedPropThreshold();

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

        visionPortal = new VisionPortal.Builder()
                .addProcessor(redProp)
                .addProcessor(apriltagProcesor)
                .setCamera(hardwareMap.get(WebcamName.class,"Camera1"))
                .setCameraResolution(new Size(640,480))
                .build();

        while(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){

        }



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


    public double Cm_To_Inch (double cm){

        return cm*0.393700787d;

    }


    public int Inch_To_Ticks(double inch){
        return (int)((inch/WHEEL_CIRCUMFERENCE)*GO_TICKS_PER_REV);
    }

}
