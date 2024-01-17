package org.firstinspires.ftc.teamcode.Experimental;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Implementations.Annotations.Experimental;
import org.firstinspires.ftc.teamcode.Implementations.Math.MathFunc;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.IOException;
import java.util.List;

@Photon
@Config
@TeleOp(name = "PID Atag Strafe")

public class PID_Atag_Strafe extends OpMode {

    public MathFunc matee;


    public FtcDashboard dashboard;
    private PIDController controller;

    public static double p=0d, i=0d, d=0d;

    private static double target=0;

    public double val=0;

    private Robot robot;
    public static int tagID = 0;
    public static int distance = 0;

    private List<LynxModule> allHubs;

    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        try {
            robot = new Robot(hardwareMap,telemetry,-1);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        dashboard=FtcDashboard.getInstance();
        controller=new PIDController(p,i,d);
        telemetry=new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    }


    @Experimental()
    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }


        controller.setPID(p,i,d);

        target=robot.move.returnXPoseError(tagID,robot,robot.camera.atag);

       // target=matee.inchToTicksD(target);



        double power;


        if(target==0){

            power=0;

        }else{
            double pid=controller.calculate(target, distance);

            power = pid;

        }


        robot.wheels.setPower(-power,power,power,-power);

        //telemetry.addData("target ", target);
        //telemetry.update();

        if(robot.move.returnAprilTAg(tagID,robot,robot.camera.atag)!=null){

            AprilTagDetection detection=robot.move.returnAprilTAg(tagID,robot,robot.camera.atag);

            telemetry.addLine("XPose: "+detection.ftcPose.x);
            telemetry.addLine("YPose: "+detection.ftcPose.y);
            telemetry.addLine("ZPose: "+detection.ftcPose.z);

            telemetry.addLine("Range: "+detection.ftcPose.range);
            telemetry.addLine("Bearing: "+detection.ftcPose.bearing);
            telemetry.addLine("Elevation: "+detection.ftcPose.elevation);

            telemetry.addLine("Yaw: "+detection.ftcPose.yaw);
            telemetry.addLine("Pitch: "+detection.ftcPose.pitch);
            telemetry.addLine("Roll: "+detection.ftcPose.roll);


        }

        telemetry.update();


    }



}
