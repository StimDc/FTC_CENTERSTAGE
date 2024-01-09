package org.firstinspires.ftc.teamcode.Implementations.Robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Robot{
    public Wheels wheels;
    public Movement move;

    public Servo joint,claw;
    public double HEADING;
    //HardwareMap hardwareMap;
    public Camera camera;
    public Arm arm;
    IMU imu;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry,int startPosition){ // startPosition 1 redFrontstage,-1 redbackstage, 2 bluefrontstage, -2 bluebackstage
        this.arm = new Arm(hardwareMap,telemetry);
        this.camera = new Camera(hardwareMap,startPosition);
        //this.hardwareMap = hardwareMap;
        this.InitIMU(hardwareMap);
        this.wheels = new Wheels(hardwareMap);
        this.move = new Movement();
        this.move.init(hardwareMap,this.wheels,this.imu,telemetry);

        this.joint = hardwareMap.get(Servo.class, "joint");
        this.claw = hardwareMap.get(Servo.class, "claw");


    }

    public void InitIMU(HardwareMap hardwareMap){
        this.imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        this.imu.initialize(parameters);
        this.imu.resetYaw();
        this.HEADING = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }




}
