package org.firstinspires.ftc.teamcode.Implementations.Robot;


import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.ChoiceMenu.ChoiceMenu;

import java.io.IOException;
import java.util.Dictionary;
import java.util.List;

public class Robot{
    public Wheels wheels;
    public Movement move;

    public Servo joint,claw;
    public double HEADING;
    public Camera camera;
    public Arm arm;
    List<LynxModule> lynxModules;
    IMU imu;

    public Dictionary<String,String> parameter;
    public LynxModule controlHub, expansionHub;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) throws IOException { // startPosition 1 redFrontstage,-1 redbackstage, 2 bluefrontstage, -2 bluebackstage
        this.parameter = ChoiceMenu.readFromFile();
        this.arm = new Arm(hardwareMap,telemetry);
        this.camera = new Camera(hardwareMap,this);
        this.InitIMU(hardwareMap);
        this.wheels = new Wheels(hardwareMap);
        this.move = new Movement();
        this.move.init(hardwareMap,this.wheels,this.imu,telemetry);

        this.joint = hardwareMap.get(Servo.class, "joint");
        this.claw = hardwareMap.get(Servo.class, "claw");

        this.lynxModules = hardwareMap.getAll(LynxModule.class);

        for(LynxModule module : this.lynxModules){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        if(this.lynxModules.get(0).isParent() && LynxConstants.isEmbeddedSerialNumber(this.lynxModules.get(0).getSerialNumber())){
            this.controlHub = this.lynxModules.get(0);
            this.expansionHub = this.lynxModules.get(1);


        }
        else{
            this.controlHub = this.lynxModules.get(1);
            this.expansionHub = this.lynxModules.get(0);
        }

        this.controlHub.setConstant(Color.rgb(255,0,0));
        this.controlHub.setConstant(Color.rgb(255,100,20));


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
