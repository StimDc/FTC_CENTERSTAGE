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
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.Implementations.Math.MathFunc.MaxPower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

@Photon
@Autonomous(name="April Move", group = "Robot")

public class April_Move extends LinearOpMode {

    private  Robot robot;

    private int tagID=6;

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



    @Override
    public void runOpMode() {

        try {
            robot = new Robot(hardwareMap,telemetry);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        robot.camera.openBackCam();

        dashboard=FtcDashboard.getInstance();

        forward=new PIDController(Pf,If,Df);
        forward.setPID(Pf,If,Df);

        strafe=new PIDController(Ps,Is,Ds);
        strafe.setPID(Ps,Is,Ds);

        turn=new PIDController(Pt,It,Dt);
        turn.setPID(Pt,It,Dt);

        telemetry=new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        setManualExposure(1,1);

        waitForStart();

     //   robot.wheels.reverseDirection();
        robot.wheels.setDirection();

        while(opModeIsActive() && !isStopRequested()){

            if(robot.move.anyAprilTags(robot,robot.camera.atag)){

                robot.move.showAprilTags(robot,robot.camera.atag);

                Go_to_April();

            }else{

                telemetry.addLine("No april tags :/");

            }


            telemetry.update();

        }

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

    public void AprilPID(){

        double powerForward=ForwardPID();
        double powerStrafe=StrafePID();
        double powerTurn=TurnPID();

        double powerFrontLeft=-powerForward-powerStrafe-powerTurn;
        double powerFrontRight=-powerForward+powerStrafe+powerTurn;
        double powerBackLeft=-powerForward+powerStrafe-powerTurn;
        double powerBackRight=-powerForward-powerStrafe+powerTurn;

        double maxPower=MaxPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);

        if(maxPower>1){

            powerFrontLeft/=maxPower;
            powerFrontRight/=maxPower;
            powerBackLeft/=maxPower;
            powerBackRight/=maxPower;
        }

        maxPower=MaxPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);

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

        boolean done=false;

        while(!done){

            AprilTagDetection detection=robot.camera.returnAprilTAg(tagID);

            if(detection!=null){

                if(detection.ftcPose.range>=Distancef-2 && detection.ftcPose.range<=Distancef+2 && detection.ftcPose.bearing>=Distancet-7 && detection.ftcPose.bearing<=Distancet+7 && detection.ftcPose.yaw>=Distances-7 && detection.ftcPose.yaw<=Distances+7){

                    hope=1;
                    done=true;
                    robot.wheels.setPower(0,0,0,0);

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

                robot.wheels.setPower(0,0,0,0);


            }

        }


    }

    private boolean   setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (robot.camera.vision == null) {
            return false;
        }

        // Wait for the camera to be open
        if (robot.camera.vision.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((robot.camera.vision.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!false)
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = robot.camera.vision.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(7, TimeUnit.MILLISECONDS);
            sleep(20);

            telemetry.addLine("Exposure Ceva: "+exposureControl);

            // Set Gain.
            GainControl gainControl = robot.camera.vision.getCameraControl(GainControl.class);

            telemetry.addLine("Gain Ceva: "+gainControl);

            if(gainControl!=null){
                boolean haide= gainControl.setGain(255);

            }
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }




}



