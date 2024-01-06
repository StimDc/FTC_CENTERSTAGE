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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;

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

@Config
@Autonomous(name="Arm State Machine", group = "Robot")

public class Arm_State_Machine extends LinearOpMode {

    private DcMotorEx elevator1, elevator2;
    private PIDController controller;

    public static double p=0.01d, i=0, d=0.006d;//d=0.004d
    public static double f=0.04d;

    public static int target=0;
    public static int tolerance=5;

    public double val=0;
    private final double ticks_in_degrees=288/(360.0*0.36); /// gear ratio: 45/125=0.36

    Servo claw,joint;


    int state=0;

    boolean OKtarget=false;


    @Override
    public void runOpMode() {

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        InitArm();

        claw=hardwareMap.get(Servo.class,"claw");
        joint=hardwareMap.get(Servo.class,"joint");

        waitForStart();

        /*
        while(opModeIsActive() && !isStopRequested()){

            ShowPos();

        }

         */

        double ceva=0;

        joint.setPosition(Joint.UP);
        claw.setPosition(Claw.CLOSED);

        sleep(1500);

        /*

        while(opModeIsActive() && !isStopRequested()){
            ShowPos();


        }


         */



        while(opModeIsActive() && !isStopRequested()){

            switch (state){

                case 0:

                    setPosition(0,1);
                    state=1;
                    break;

                case 1:
                    setPosition(381,1);
                    state=2;
                    break;

                case 2:
                    if(isOnTarget()) {
                        state=3;
                    }
                    break;

                case 3:

                    claw.setPosition(Claw.OPEN);
                    state=4;
                    break;

                case 4:


                    if(Math.abs(claw.getPosition()-Claw.OPEN) <0.03){

                       state=5;
                    }
                    break;


                case 5:
                    setPosition(0,1);
                    state=6;
                    break;

                case 6:
                    if(isOnTarget()) {
                        if(target==0){

                            OKtarget=true;

                        }
                        state=7;
                    }
                    break;

                case 7:

                    telemetry.addLine("DONE :D");
                   // telemetry.update();

            }

            if(target==0 && OKtarget==true){

                elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                setPosition(0,0);
                ArmTask();

            }

            if(target>0){
                OKtarget=false;

            }

            if(OKtarget==false){
                ArmTask();

            }
            telemetry.addLine("Ceva: "+ceva);
            telemetry.addLine("Pos: "+elevator1.getCurrentPosition());
            telemetry.addLine("Target: "+target);
            telemetry.addLine("State: "+state);
            telemetry.update();

        }







    }

    public void InitArm()
    {
        controller=new PIDController(p,i,d);
        controller.setPID(p,i,d);

        elevator1=hardwareMap.get(DcMotorEx.class,"e1");
        elevator2=hardwareMap.get(DcMotorEx.class,"e2");

        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void ArmTask(){

        int elepos=elevator1.getCurrentPosition();

        double pid=controller.calculate(elepos, target);
        double ff=Math.cos(Math.toRadians(target/ticks_in_degrees))*f;

        double power = pid + ff;

        elevator1.setPower(power);
        elevator2.setPower(power);

    }

    public boolean isOnTarget()
    {
        //double currPosInDegrees = arm.getCurrentPosition() / ARM_TICKS_PER_DEGREE;


        double currPosInDegrees = elevator1.getCurrentPosition();

        return Math.abs(target- currPosInDegrees) <= tolerance;
    }

    public void ShowPos(){

        telemetry.addLine("Pos: "+elevator1.getCurrentPosition());
        telemetry.update();

    }

    public void setPosition(int targetPosInDegrees, int toleranceInDegrees)
    {
        target= targetPosInDegrees;
        tolerance = toleranceInDegrees;
    }
}
