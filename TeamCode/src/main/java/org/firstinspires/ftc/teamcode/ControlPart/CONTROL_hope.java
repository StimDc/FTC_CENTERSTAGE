package org.firstinspires.ftc.teamcode.ControlPart;

import static java.lang.Math.abs;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.Experimental;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Wheels;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp(name = "Hope Control")

public class CONTROL_hope extends OpMode {

    private Servo joint, claw;

    private Claw clawpos;

    private Joint jointpos;
    private boolean OKJoint=true,OKClaw=true, PressClaw=false, PressJoint=false;

    private boolean once=false;

    static final double CLAWCLOSED=0.655444d, CLAWOPEN=0.377222d, CLAWINTERMEDIARY=0.555d;
    static final double JOINTUP=1, JOINTDOWN=0.119333d;

    static final double AVIONSTART=0, AVIONRELEASE=0.5d;


    private DcMotorEx viper,hang;

    private DcMotor frontLeft,frontRight, backLeft, backRight;

    public Wheels wheels;

    private  final double GO_TICKS_117=1425.1d;

 ///PID ARM
    private DcMotorEx  elevator1, elevator2;
    private PIDController controller;

    public static double p=0.03, i=0, d=0.0006;
    public static double f=0.04;

    public static int target=0;

    public double val=0;


    private final double ticks_in_degrees=288/(360.0*0.36); /// gear ratio: 45/125=0.36

    private VisionPortal camBack;
    private AprilTagProcessor apriltagProcesor;

    private AprilTagDetection desiredTag;



    @Override
    public void init() {

        InitCamera();
        controller=new PIDController(p,i,d);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        clawpos=new Claw();
        jointpos=new Joint();

        wheels = new Wheels(hardwareMap);
        wheels.setDirection();

        claw=hardwareMap.get(Servo.class,"claw");
        joint=hardwareMap.get(Servo.class,"joint");


        elevator1=hardwareMap.get(DcMotorEx.class,"e1");
        elevator2=hardwareMap.get(DcMotorEx.class,"e2");

        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        viper=hardwareMap.get(DcMotorEx.class,"v");
        viper.setDirection(DcMotorSimple.Direction.FORWARD);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        hang=hardwareMap.get(DcMotorEx.class,"s");
        hang.setDirection(DcMotorSimple.Direction.FORWARD);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    @Experimental()
    @Override
    public void loop() {

        if(!once){
            joint.setPosition(jointpos.UP);
            claw.setPosition(clawpos.INTERMEDIARY);//CLAWINTERMEDIARY
            once=true;
        }

        if(gamepad2.b){
            if(!OKJoint && !PressJoint){
                joint.setPosition(jointpos.UP);
                OKJoint=true;

            }
            else if(OKJoint && !PressJoint){
                joint.setPosition(jointpos.DOWN);
                OKJoint=false;

            }
            PressJoint=true;

        }else {
            PressJoint=false;
        }


        if(gamepad2.dpad_right){
            joint.setPosition(0.4);
            OKJoint=false;
        }

        if(gamepad2.dpad_left){
            claw.setPosition(clawpos.OPEN);//CLAWOPEN
            OKClaw=false;
        }

        if(gamepad2.a){
            if(!OKClaw && !PressClaw){
                claw.setPosition(clawpos.INTERMEDIARY);//CLAWINTERMEDIARY
                OKClaw=true;

            }
            else if(OKClaw && !PressClaw){
                claw.setPosition(clawpos.CLOSED);//CLAWCLOSED
                OKClaw=false;

            }
            PressClaw=true;
        }else{
            PressClaw=false;
        }

        if(gamepad2.right_stick_y<0){


                hang.setPower(-gamepad2.right_stick_y);



        }else if(gamepad2.right_stick_y>0){

            if(gamepad2.right_stick_y>0.25){
                hang.setPower(0.25);
            }else{
                hang.setPower(-gamepad2.right_stick_y);
                
            }

        }else if(gamepad2.right_stick_y<0){

            hang.setPower(-0.25);

        }else if(gamepad2.y && viper.getCurrentPosition()<1620){

            viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            viper.setPower(0.2);
            hang.setPower(-0.5);

        }
        else if(gamepad2.x && viper.getCurrentPosition()>0){

            viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            viper.setPower(-0.3);

        }else{

            hang.setPower(0);
            viper.setPower(0);
            viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        double drive =0;
        double strafe=0;
        double turn =0;

      /*NORMAL METHOD
        drive=-gamepad1.left_stick_y/2.0;
        strafe=-gamepad1.left_stick_x/2.0;
        turn=-gamepad1.right_stick_x/3.0;
       */


        boolean targetFound = false;
        desiredTag  = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = apriltagProcesor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && (detection.id==5 || detection.id==2)){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }

        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (gamepad1.y && targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - 5);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * 1, -0.7, 0.7);
            turn   = Range.clip(headingError * 1.1, -0.7, 0.7) ;
            strafe = Range.clip(-yawError * 1, -0.7, 0.7);

            drive*=(-1);
            turn*=(-1);
            strafe*=(-1);
        }else if(gamepad1.a){

            drive=-gamepad1.left_stick_y/1.25;

        }else if(gamepad1.b){

            strafe=-gamepad1.left_stick_x*1.1/1.25;

        }else {

            drive=-gamepad1.left_stick_y/1.25;
            strafe=-gamepad1.left_stick_x*1.1/1.25;
            turn=-gamepad1.right_stick_x/2.5;
        }

        if(gamepad1.x && targetFound){

            double  headingError    = desiredTag.ftcPose.yaw;
            turn   = Range.clip(headingError * 1, -0.7, 0.7) ;

        }

        moveRobot(drive,strafe,turn);
        moveElevator();


    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x -y -yaw;
        double frontRightPower   =  x +y +yaw;
        double backLeftPower     =  x +y -yaw;
        double backRightPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.max(Math.max(frontLeftPower,backLeftPower), backRightPower),frontRightPower);

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        wheels.setPower(frontLeftPower,frontRightPower,backLeftPower,backRightPower);

    }

    public void moveElevator(){

        controller.setPID(p,i,d);
        int elepos=elevator1.getCurrentPosition();

        double pid=controller.calculate(elepos, target);
        double ff=Math.cos(Math.toRadians(target/ticks_in_degrees))*f;

        double power = pid + ff;

        elevator1.setPower(power);
        elevator2.setPower(power);

        telemetry.addData("pos ",elepos);
        telemetry.addData("target ", target);
        telemetry.update();

        if(gamepad2.left_trigger>0){
            val+=gamepad2.left_trigger*0.75;
        }

        if(gamepad2.right_trigger>0 && val>0){
            val-=gamepad2.right_trigger*1.25;
        }

        target=(int) val;

        if(target==0 && elepos>=-5 && elepos<=5){
            elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            elevator1.setPower(0);
            elevator2.setPower(0);
        }

        target=(int) val;

    }

    public void InitCamera(){

        apriltagProcesor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                //.3 .setLensIntrinsics(1731.46, 1731.46, 119.867, 12.6661)

                /*
                NU BUN 640x480 Camera logitech: 1731.46, 1731.46, 119.867, 12.6661
                640x480 Camera No-name:  2475.88, 2475.88, 249.071, 110.786
                 */
                .build();


        camBack=new VisionPortal.Builder()
                .addProcessor(apriltagProcesor)
                .setCamera(hardwareMap.get(WebcamName.class,"Camera1"))
                .setCameraResolution(new Size(640,480))
                .build();


        while(camBack.getCameraState() != VisionPortal.CameraState.STREAMING){

        }



         /* CA SA REDUCEM BLURAREA APRIL TAG-ULUI, ATUNCI CAND SE MISCA ROBOTUL

        ExposureControl exposure =myVisionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);
        /// telemetry.addData("Exposure: ",exposure.isExposureSupported());


        GainControl gain=myVisionPortal.getCameraControl(GainControl.class);
        gain.setGain(255);
        ///telemetry.addData("Min gain: ", gain.getMinGain());
        ///telemetry.addData("Max gain: ",gain.getMaxGain());


         */

    }

    }


