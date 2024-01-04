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

package org.firstinspires.ftc.teamcode.ControlPart;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;


@Config
@TeleOp(name="Hope Arm Calibration", group = "Hope")

public class Hope_ARM_PID_Cali extends OpMode {

    private PIDController controller;

    public static double p=0, i=0, d=0;
    public static double f=0;
    //public static int target=0;


    private static final double MOTOR_CPR = 288.0;
    private static final double GEAR_RATIO = 125.0 / 45.0;
    private static final double ARM_TICKS_PER_DEGREE = MOTOR_CPR * GEAR_RATIO / 360.0;
   // private static final double MAX_ARM_HOLDING_POWER = <some calibrated value here>;
    private static final double ZERO_OFFSET = 70.0-3.85;
    private DcMotorEx elevator1, elevator2;
    public static double targetPosInDegrees=70.0-3.85;
    private double powerLimit;

    private  Servo joint;

    private boolean once=false;

    public void init()
    {
        controller=new PIDController(p,i,d);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elevator1= hardwareMap.get(DcMotorEx.class,"e1");
        elevator2= hardwareMap.get(DcMotorEx.class,"e2");


        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        joint =hardwareMap.get(Servo.class,"joint");
    }

    public void loop(){

        controller.setPID(p,i,d);

        if(!once){
            joint.setPosition(Joint.UP);
            once=true;
        }

       // if(isOnTarget(1)) {

            setPosition(targetPosInDegrees,0.7);

       // }

        armTask();

        telemetry.addData("pos ",getPosition());
        telemetry.addData("target ", targetPosInDegrees);
        telemetry.update();

    }

    public void armTask()
    {
        double targetPosInTicks = (targetPosInDegrees - ZERO_OFFSET) * ARM_TICKS_PER_DEGREE;
        double currPosInTicks = elevator1.getCurrentPosition();
        double pidOutput = controller.calculate(currPosInTicks, targetPosInTicks);
        // This ff is assuming arm at horizontal position is 90-degree.
        double ff = f * Math.sin(Math.toRadians(ticksToRealWorldDegrees(currPosInTicks)));
        double power = pidOutput + ff;
        // Clip power to the range of -powerLimit to powerLimit.
        power = power < -powerLimit ? -powerLimit : power > powerLimit ? powerLimit : power;
        elevator1.setPower(power);
        elevator2.setPower(power);
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

    //
// In your OpMode
//
    static final double ARM_HOME_POS = 45.0;
    static final double ARM_LOADING_POS = 50.0;
    static final double ARM_SCORING_POS = 270.0;
 //   Arm arm;
  //  Elevator elevator;
  //  DriveBase driveBase;
    int state = 0;

}