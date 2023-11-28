
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold;
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

@Autonomous(name="Primitiva SMART", group = "Autonomie Primitiva")

public class Autonomie_PrimitiveSMART extends LinearOpMode {

    private RedPropThreshold redProp;
    private VisionPortal visionPortal;
    private AprilTagProcessor apriltagProcesor;
    private int apriltagid;
    private double distx=999, disty=999;


    private static final double PI = 3.14159265d;
    private static final double GO_TICKS_PER_REV = 537.7d;
    private DcMotor frontLeft,frontRight, backLeft, backRight;
    private static final double  WHEEL_CIRCUMFERENCE =  PI * 3.7795275d;
    private final double POWER=0.8;

    private IMU imu;
    private double HEADING;

    static final double JOINTUP=1, JOINTDOWN=0.131555d;
    private boolean once=false;
    private Servo joint, claw;


    @Override
    public void runOpMode() {

        // InitCamera();
        InitMotors();
        InitIMU();

        joint=hardwareMap.get(Servo.class,"joint");

        telemetry.addLine("Heading"+ imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();

        waitForStart();

        if(!once){
            joint.setPosition(JOINTUP);
            once=true;
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

        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        int ticks=(int)((Inch_To_Ticks(Cm_To_Inch(dist))*(1/Math.sin(45) ) )*35/42);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setTargetPosition(-ticks*sign);
        frontLeft.setTargetPosition(ticks*sign);
        backRight.setTargetPosition(ticks*sign);
        backLeft.setTargetPosition(-ticks*sign);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower(pow);
        frontLeft.setPower(pow);
        backRight.setPower(pow);
        backLeft.setPower(pow);

        while(frontRight.isBusy() && frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy()){

        }

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void Strafing(int sign,double pow, double dist){

        if(sign>1){
            sign=1;
        }
        if(sign<-1){
            sign=-1;
        }

        dist=(int)Inch_To_Ticks(Cm_To_Inch(dist));

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double x=dist-(int)Inch_To_Ticks(Cm_To_Inch(5));

        while(Math.abs(frontLeft.getCurrentPosition())<Math.abs(x)&& Math.abs(frontRight.getCurrentPosition())<Math.abs(x) && Math.abs(backRight.getCurrentPosition())<Math.abs(x) && Math.abs(backLeft.getCurrentPosition())<Math.abs(x)){

            frontRight.setPower(-sign* pow);
            frontLeft.setPower(sign* pow);
            backRight.setPower(sign* pow);
            backLeft.setPower(-sign* pow);

        }

        while(Math.abs(frontLeft.getCurrentPosition())<Math.abs(dist)&& Math.abs(frontRight.getCurrentPosition())<Math.abs(dist) && Math.abs(backRight.getCurrentPosition())<Math.abs(dist) && Math.abs(backLeft.getCurrentPosition())<Math.abs(dist)){

            if(pow>0.1){
                pow=pow-0.01;
            }

            frontRight.setPower(-sign* pow);
            frontLeft.setPower(sign* pow);
            backRight.setPower(sign* pow);
            backLeft.setPower(-sign* pow);

        }

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        int ticks=(int)(Inch_To_Ticks(Cm_To_Inch(dist))*35/42);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight.setTargetPosition(ticks*sign);
        frontLeft.setTargetPosition(ticks*sign);
        backRight.setTargetPosition(ticks*sign);
        backLeft.setTargetPosition(ticks*sign);

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower(pow);
        frontLeft.setPower(pow);
        backRight.setPower(pow);
        backLeft.setPower(pow);

        double x=(int)Inch_To_Ticks(Cm_To_Inch(10*(pow-0.6)*10*pow/0.6));

        while(frontRight.isBusy() && frontLeft.isBusy() && backRight.isBusy() && backLeft.isBusy()){

            if(Math.abs(frontRight.getCurrentPosition())>Math.abs(x) && Math.abs(frontLeft.getCurrentPosition())>Math.abs(x) && Math.abs(backRight.getCurrentPosition())>Math.abs(x) && Math.abs(backLeft.getCurrentPosition())>Math.abs(x)){

                if(pow>0.01){
                    pow=pow-0.4;
                }
                if(pow<0.01){
                    pow=0.01;
                }

                frontRight.setPower(sign* pow);
                frontLeft.setPower(sign* pow);
                backRight.setPower(sign* pow);
                backLeft.setPower(sign* pow);

            }

        }

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

      /*  frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       */

    }

    public void Forward(int sign,double pow, double dist){

        if(sign>1){
            sign=1;
        }
        if(sign<-1){
            sign=-1;
        }

        dist=(int)(Inch_To_Ticks(Cm_To_Inch(dist)));

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double x=dist-(int)Inch_To_Ticks(Cm_To_Inch(10*(pow-0.6)*10*pow/0.6));

        while(Math.abs(frontLeft.getCurrentPosition())<Math.abs(x)&& Math.abs(frontRight.getCurrentPosition())<Math.abs(x) && Math.abs(backRight.getCurrentPosition())<Math.abs(x) && Math.abs(backLeft.getCurrentPosition())<Math.abs(x)){

            frontRight.setPower(sign* pow);
            frontLeft.setPower(sign* pow);
            backRight.setPower(sign* pow);
            backLeft.setPower(sign* pow);

        }

        while(Math.abs(frontLeft.getCurrentPosition())<Math.abs(dist)&& Math.abs(frontRight.getCurrentPosition())<Math.abs(dist) && Math.abs(backRight.getCurrentPosition())<Math.abs(dist) && Math.abs(backLeft.getCurrentPosition())<Math.abs(dist)){

            if(pow>0.01){
                pow=pow-0.4;
            }
            if(pow<0.01){
                pow=0.01;
            }

            frontRight.setPower(sign* pow);
            frontLeft.setPower(sign* pow);
            backRight.setPower(sign* pow);
            backLeft.setPower(sign* pow);

        }

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        double botHeading=Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))-Math.abs(HEADING);
        deg=Math.toRadians(deg)*(90/130);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while(Math.abs(botHeading)<Math.abs(deg)){

            frontLeft.setPower(pow*sign);
            frontRight.setPower(-pow*sign);
            backLeft.setPower(pow*sign);
            backRight.setPower(-pow*sign);

            botHeading = Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))-Math.abs(HEADING);

            telemetry.addLine("Heading"+ imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        HEADING=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        telemetry.addLine("Heading"+ imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();

    }

    public double Cm_To_Inch (double cm){

        double inch=cm*0.393700787d;

        return inch;
    }


    public int Inch_To_Ticks(double inch){
        return (int)((inch/WHEEL_CIRCUMFERENCE)*GO_TICKS_PER_REV);
    }


}
