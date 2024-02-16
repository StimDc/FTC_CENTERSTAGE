package org.firstinspires.ftc.teamcode.ControlPart;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Wheels;

@TeleOp(name = "FIFA Control")
@Disabled

public class FIFA_Control extends OpMode {


    IMU imu;


    static final double JOINTUP=1, JOINTDOWN=0.131555d;
    private boolean once=false;
    private Servo joint, claw;

    private Wheels wheels; //MOTORS

    @Override
    public void init() {

        wheels = new Wheels(hardwareMap);
        wheels.setDirection();

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        joint=hardwareMap.get(Servo.class,"joint");


    }


    @Override
    public void loop() {

        if(!once){
            joint.setPosition(JOINTUP);
            once=true;
        }

            double y = -gamepad1.left_stick_y/1.5; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x/1.5;
            double rx = gamepad1.right_stick_x/3;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            wheels.setPower(frontLeftPower,frontRightPower,backLeftPower,backRightPower);

            telemetry.addLine("Heading: "+imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

    }

}
