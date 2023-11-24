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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous(name="AprilTag Move", group = "Robot")

public class AprilTag_Move extends LinearOpMode {

    private static final double GO_TICKS_PER_REV = 537.7d;

   /// private static final double CONST_LATERAL_MOVEMENT = 1.136363636363d;


    private static final double PI = 3.14159265d;
    private static final double  WHEEL_CIRCUMFERENCE =  PI * 3.7795275d;

    private AprilTagProcessor apriltagProcesor;

    private VisionPortal myVisionPortal;

    private int apriltagid;

    private double distx=999, disty=999;

    private DcMotor frontLeft,frontRight, backLeft, backRight;

    private final double POWER=0.3;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class, "FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        /// apriltagProcesor = AprilTagProcessor.easyCreateWithDefaults();
        /// myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Camera1"), apriltagProcesor);


        apriltagProcesor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
               // .setLensIntrinsics()
                .build();

        myVisionPortal = new VisionPortal.Builder()
                .addProcessor(apriltagProcesor)
                .setCamera(hardwareMap.get(WebcamName.class,"Camera1"))
                .setCameraResolution(new Size(640,480))
                ///.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();


        while(myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){

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


        waitForStart();

        while(distx == 999 && disty==999) {
            AprilTagg_Detection();
        }

        while (!isStopRequested() && opModeIsActive()) {




            lateral(POWER,Inch_To_Ticks(distx *(1/ Math.sin(45))));
            distx=0;

            sleep(2500);

            GoFront(POWER,Inch_To_Ticks(disty-9.5));
            disty=9.5;

            sleep(2500);

        }

    }


    public void lateral (double pow, int dist){

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        frontRight.setTargetPosition(-dist);
        frontLeft.setTargetPosition(dist);
        backRight.setTargetPosition(dist);
        backLeft.setTargetPosition(-dist);

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

    }

    public void GoFront(double pow, int dist){

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        frontRight.setTargetPosition(dist);
        frontLeft.setTargetPosition(dist);
        backRight.setTargetPosition(dist);
        backLeft.setTargetPosition(dist);

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

    }

    public void AprilTagg_Detection(){

        Vector3 april1 = new Vector3();
        Vector3 april2 = new Vector3();
        Vector3 april3 = new Vector3();


        List<AprilTagDetection> AprilDetections;

        AprilDetections = apriltagProcesor.getDetections();

        telemetry.addData("April Tag-uri: ",AprilDetections.size());


        for (AprilTagDetection currentDetection : AprilDetections) {

            if (currentDetection.metadata != null) {
                apriltagid = currentDetection.id;

                switch(apriltagid){
                    case 1:
                        april1.setVector(currentDetection.ftcPose.x, currentDetection.ftcPose.y, currentDetection.ftcPose.z);

                        break;
                    case 2:
                        april2.setVector(currentDetection.ftcPose.x, currentDetection.ftcPose.y, currentDetection.ftcPose.z);
                        distx=april2.x;
                        disty=april2.y;

                        break;
                    case 3:
                        april3.setVector(currentDetection.ftcPose.x, currentDetection.ftcPose.y, currentDetection.ftcPose.z);
                        break;
                }
                telemetry.addLine("AprilTag1: X " + String.valueOf(april1.x) + " Y " + String.valueOf(april1.y) + " Z " + String.valueOf(april1.z));
                telemetry.addLine("AprilTag2: X " + String.valueOf(april2.x) + " Y " + String.valueOf(april2.y) + " Z " + String.valueOf(april2.z));
                telemetry.addLine("AprilTag3: X " + String.valueOf(april3.x) + " Y " + String.valueOf(april3.y) + " Z " + String.valueOf(april3.z));





                telemetry.update();
                sleep(20);

            }
        }



    }


    public double Cm_To_Inch (int cm){

        double inch=cm*0.393700787;

        return inch;
    }


    public int Inch_To_Ticks(double inch){
        return (int)((inch/WHEEL_CIRCUMFERENCE)*GO_TICKS_PER_REV);
    }


}
