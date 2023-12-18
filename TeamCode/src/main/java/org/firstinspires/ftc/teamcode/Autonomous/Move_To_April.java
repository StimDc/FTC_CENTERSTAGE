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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold;
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

@Autonomous(name="Move to April", group = "Robot")

public class Move_To_April extends LinearOpMode {

    private WebcamName webcam1, webcam2;

    private RedPropThreshold redProp;

    private BluePropThreshold blueProp;

    private AprilTagProcessor atag;


    private VisionPortal visionPortal;


    private static final double PI = 3.14159265d;
    private static final double GO_TICKS_PER_REV = 537.7d;
    private Wheels wheels;
    private static final double  WHEEL_CIRCUMFERENCE =  PI * 3.7795275d;
    public static double POWER;

    public static double rangeErrorGAIN=0.02, headingErrorGAIN=0.01,yawErrorGAIN=0.015;

    public static double MAXrange=0.3,MAXheading=0.2,MAXyaw=0.2;
    @Override
    public void runOpMode() {

        Init_Cameras();
        InitMotors();

        waitForStart();


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

    public void Init_Cameras(){

        atag= new AprilTagProcessor.Builder().build();

        redProp=new RedPropThreshold();
        blueProp=new BluePropThreshold();

        webcam1 = hardwareMap.get(WebcamName.class, "Camera1");
        webcam2 = hardwareMap.get(WebcamName.class, "Camera2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(atag,redProp,blueProp)
                .build();

        while(visionPortal.getCameraState()!= VisionPortal.CameraState.STREAMING){

        }

        visionPortal.setProcessorEnabled(redProp,false);
        visionPortal.setProcessorEnabled(blueProp,false);

    }

    public void Open_Cam1(){

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.setActiveCamera(webcam1);
        }

        visionPortal.setProcessorEnabled(atag,true);
        visionPortal.setProcessorEnabled(redProp,false);
        visionPortal.setProcessorEnabled(blueProp,false);

    }

    public void Open_Cam2(){

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.setActiveCamera(webcam2);
        }

        visionPortal.setProcessorEnabled(atag,false);
        visionPortal.setProcessorEnabled(redProp,true);
        visionPortal.setProcessorEnabled(blueProp,true);

    }


    public void InitMotors(){
        wheels = new Wheels(hardwareMap);
        wheels.setDirection();
        wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

}
