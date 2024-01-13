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

package org.firstinspires.ftc.teamcode.Experimental;

import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.BACKWARDS;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.FORWARD;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;

import java.io.IOException;

@Autonomous(name="testare", group = "Robot")
public class TestareFunctiiNoi extends LinearOpMode {

    Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {

    }


    @Config
@Autonomous(name="Moving Claw", group = "Robot")

public class MovingClaw extends LinearOpMode {


    @Override
    public void runOpMode() {
        try {
            robot = new Robot(hardwareMap,telemetry,1);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        robot.move.forward( FORWARD,0.6,30); //TODO: REFACUT ROTATIILE
        //sleep(300);
        robot.move.forward(BACKWARDS,0.6,30);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        robot.claw.setPosition(Claw.CLOSED);
        sleep(1000);
        while(opModeIsActive() && !isStopRequested()){
            /*
            if(claw.getPosition()>= Claw.OPEN-0.002 || claw.getPosition()<=Claw.OPEN+0.002){
                claw.setPosition(Claw.CLOSED );
                sleep(5000);
            }else if(claw.getPosition()>=Claw.CLOSED-0.002 || claw.getPosition()<=Claw.CLOSED+0.002){
                claw.setPosition(Claw.OPEN);
                sleep(5000);
            }

             */

            if(Math.abs(robot.claw.getPosition()-Claw.OPEN) <0.002){
                robot.claw.setPosition(Claw.CLOSED);
            }else if(Math.abs(robot.claw.getPosition()-Claw.CLOSED) <0.002){
                robot.claw.setPosition(Claw.OPEN);
            }
            sleep(1500);

            telemetry.addLine("Pos: "+ robot.claw.getPosition());
            telemetry.update();

        }

    }


}}
