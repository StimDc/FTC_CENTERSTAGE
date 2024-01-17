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

import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
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
import org.firstinspires.ftc.teamcode.Implementations.Constants.PIDConstantsArm;

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
@Photon
@Config
@Autonomous(name="Arm State Machine", group = "Robot")

public class Arm_State_Machine extends LinearOpMode {

    public  final double ZERO_OFFSET = 70.0-3.85;

    private DcMotorEx elevator1, elevator2;
    private PIDController controller;

    public static double p=0d, i=0d, d=0d;//d=0.004d
    public static double f=0d;
    public static double target=70.0-3.85;

    private static final double MOTOR_CPR = 288.0;
    private static final double GEAR_RATIO = 125.0 / 45.0;
    private static final double ARM_TICKS_PER_DEGREE = MOTOR_CPR * GEAR_RATIO / 360.0;
    // private static final double MAX_ARM_HOLDING_POWER = <some calibrated value here>;
    private  double targetPosInDegrees=70.0-3.85;
    public static double powerLimit=0.7;

    Servo claw,joint;


    int state=0;

    boolean OKtarget=false;


    @Override
    public void runOpMode() {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        InitArm();

        claw = hardwareMap.get(Servo.class, "claw");
        joint = hardwareMap.get(Servo.class, "joint");

        waitForStart();

        joint.setPosition(Joint.UP);
        claw.setPosition(Claw.CLOSED);

        int stateArm=0;

        boolean armtarget=false,OKtarget=false;







        setPosition(ZERO_OFFSET,powerLimit);

        while(opModeIsActive() && !isStopRequested()){

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            controller.setPID(p,i,d);

            if(isOnTarget(2)){

                setPosition(target,powerLimit);

            }

            armTask();
            telemetry.addLine("Target: "+target);
            telemetry.addLine("Pos: "+getPosition());
            telemetry.update();


        }

        /*

        while(!armtarget){

            switch (stateArm){

                case 0:

                    setPosition(ZERO_OFFSET,1);
                    stateArm=1;
                    break;

                case 1:
                    setPosition(240,6);
                    stateArm=2;
                    break;

                case 2:
                    if(isOnTarget(6)) {
                        stateArm=3;
                    }
                    break;

                case 3:

                    claw.setPosition(Claw.OPEN);
                    stateArm=4;
                    break;

                case 4:


                    if(Math.abs(claw.getPosition()-Claw.OPEN) <0.03){

                        stateArm=5;
                    }
                    break;


                case 5:
                    setPosition(ZERO_OFFSET,1);
                    stateArm=6;
                    break;

                case 6:
                    if(isOnTarget(5)) {
                        if(Math.abs(getPosition()-ZERO_OFFSET)<5){

                            OKtarget=true;

                        }
                        stateArm=7;
                    }
                    break;

                case 7:

                    armtarget=true;
                    telemetry.addLine("DONE :D");
                    // telemetry.update();

            }

            if(Math.abs(getPosition()-ZERO_OFFSET)<5 && OKtarget==true){

                elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                elevator1.setPower(0);
                elevator2.setPower(0);

            }

            if(Math.abs(getPosition()-ZERO_OFFSET)>=3){
                OKtarget=false;

            }

            if(OKtarget==false){
                armTask();

            }
            // telemetry.addLine("Ceva: "+ceva);
          //  telemetry.addLine("Pos: "+robot.arm.getPosition());
           // telemetry.addLine("Target: "+TargetPosInDegrees);
          //  telemetry.addLine("State: "+stateArm);
          //  telemetry.update();


        }

         */



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

        elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void armTask()
    {
        double targetPosInTicks = (targetPosInDegrees - ZERO_OFFSET) * ARM_TICKS_PER_DEGREE;
        double currPosInTicks = this.elevator1.getCurrentPosition();
        double pidOutput = this.controller.calculate(currPosInTicks, targetPosInTicks);
        // This ff is assuming arm at horizontal position is 90-degree.
        double ff = PIDConstantsArm.f * Math.sin(Math.toRadians(ticksToRealWorldDegrees(currPosInTicks)));
        double power = pidOutput + ff;
        // Clip power to the range of -powerLimit to powerLimit.
        power = power < -powerLimit ? -powerLimit : Math.min(power, powerLimit);
        this.elevator1.setPower(power);
        this.elevator2.setPower(power);
    }

    public boolean isOnTarget(double toleranceInDegrees)
    {
        double currPosInDegrees = getPosition();
        return Math.abs(targetPosInDegrees - currPosInDegrees) <= toleranceInDegrees;
    }

    public void setPosition(double targetPosInDegrees, double powerLimit)
    {
        this.targetPosInDegrees = targetPosInDegrees;
        this.powerLimit = Math.abs(powerLimit);
    }


    public double ticksToRealWorldDegrees(double ticks)
    {
        return ticks / ARM_TICKS_PER_DEGREE + ZERO_OFFSET;
    }

    public double getPosition()
    {
        return ticksToRealWorldDegrees(elevator1.getCurrentPosition());
    }
}
