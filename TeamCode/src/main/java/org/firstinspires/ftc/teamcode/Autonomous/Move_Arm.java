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

@Autonomous(name="Move Arm", group = "Robot")

public class Move_Arm extends LinearOpMode {

    private PIDController controller;

    public static double p = 0.02, i = 0, d = 0.001;
    public static double f = 0.04;

    public static int target = 0;

    public double val = 0;

    private final double ticks_in_degrees = 288 / (360.0 * 0.36); /// gear ratio: 45/125=0.36

    private DcMotorEx elevator1, elevator2;

    @Override
    public void runOpMode() {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elevator1 = hardwareMap.get(DcMotorEx.class, "e1");
        elevator2 = hardwareMap.get(DcMotorEx.class, "e2");

        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Pos: "+elevator1.getCurrentPosition());
        telemetry.addLine("Target: "+ target);
        telemetry.update();

        waitForStart();

        /*

        moveArm(50);

        sleep(5000);

        telemetry.addLine("Max pos,UP, gud");
        telemetry.update();


         */

        while(opModeIsActive() && !isStopRequested()){

            Go_To_Arm(0);

            telemetry.addLine("Down");
            telemetry.update();

        }




    }


    public void moveArm(int desiredTarget) {//TODO: RESCRIS

        if (elevator1.getCurrentPosition() < desiredTarget) {

            for (target = elevator1.getCurrentPosition(); target <= desiredTarget; target++) {
                controller.setPID(p, i, d);
                int elepos = elevator1.getCurrentPosition();
                double pid = controller.calculate(elepos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees));

                double power = pid + ff;

                elevator1.setPower(power);
                elevator2.setPower(power);
                sleep(100);

                telemetry.addLine("Pos: "+elevator1.getCurrentPosition());
                telemetry.addLine("Target: "+ target);
                telemetry.update();
            }

        } else if (elevator1.getCurrentPosition() > desiredTarget) {

            for (target = elevator1.getCurrentPosition(); target >= 0; target--) {
                controller.setPID(p, i, d);
                int elepos = elevator1.getCurrentPosition();
                double pid = controller.calculate(elepos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees));

                double power = pid + ff;

                elevator1.setPower(power);
                elevator2.setPower(power);
                sleep(100);

                telemetry.addLine("Pos: "+elevator1.getCurrentPosition());
                telemetry.addLine("Target: "+ target);
                telemetry.update();
            }

        }else{

            controller.setPID(p, i, d);
            int elepos = elevator1.getCurrentPosition();
            double pid = controller.calculate(elepos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees));

            double power = pid + ff;

            elevator1.setPower(power);
            elevator2.setPower(power);

            telemetry.addLine("Pos: "+elevator1.getCurrentPosition());
            telemetry.addLine("Target: "+ target);
            telemetry.update();

        }

    }

    public void Go_To_Arm(int dist){

        target=dist;
        controller.setPID(p, i, d);
        int elepos = elevator1.getCurrentPosition();
        double pid = controller.calculate(elepos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees));

        double power = pid + ff;

        elevator1.setPower(power);
        elevator2.setPower(power);

        telemetry.addLine("Pos: "+elevator1.getCurrentPosition());
        telemetry.addLine("Target: "+ target);
        telemetry.update();

    }

    public void PleaseArm(int desiredTarget){

        if (elevator1.getCurrentPosition() < desiredTarget) {

            for (double hope = elevator1.getCurrentPosition(); hope <= desiredTarget; hope=hope+0.225) {

                target=(int)hope;

                controller.setPID(p, i, d);
                int elepos = elevator1.getCurrentPosition();
                double pid = controller.calculate(elepos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees));

                double power = pid + ff;

                elevator1.setPower(power);
                elevator2.setPower(power);
                sleep(100);

                telemetry.addLine("Pos: "+elevator1.getCurrentPosition());
                telemetry.addLine("Target: "+ target);
                telemetry.update();
            }

        } else if (elevator1.getCurrentPosition() > desiredTarget) {

            for (double hope = elevator1.getCurrentPosition(); hope >= 0; hope=hope-0.225) {

                target=(int)hope;

                controller.setPID(p, i, d);
                int elepos = elevator1.getCurrentPosition();
                double pid = controller.calculate(elepos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees));

                double power = pid + ff;

                elevator1.setPower(power);
                elevator2.setPower(power);
                sleep(100);

                telemetry.addLine("Pos: "+elevator1.getCurrentPosition());
                telemetry.addLine("Target: "+ target);
                telemetry.update();
            }

        }else{

            target=desiredTarget;
            controller.setPID(p, i, d);
            int elepos = elevator1.getCurrentPosition();
            double pid = controller.calculate(elepos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees));

            double power = pid + ff;

            elevator1.setPower(power);
            elevator2.setPower(power);

            telemetry.addLine("Pos: "+elevator1.getCurrentPosition());
            telemetry.addLine("Target: "+ target);
            telemetry.update();

        }

    }


}
