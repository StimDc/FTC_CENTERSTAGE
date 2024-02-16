package org.firstinspires.ftc.teamcode.ControlPart;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Claw.CLOSED;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Claw.INTERMEDIARY;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Claw.OPEN;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Joint.DOWN;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Joint.UP;
import static org.firstinspires.ftc.teamcode.Implementations.Math.MathFunc.MaxPower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Implementations.Math.MathFunc;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

@Photon
@Config
@TeleOp(name = "NEW Control")

public class NEW_Control extends OpMode {



    private ElapsedTime timerr;

    private boolean once=false;
    private boolean OKJoint=true,OKClaw=true, PressClaw=false, PressJoint=false;

    private int PARKING=1; //-1 for left parking and 1 for right
    //  private BluePropThreshold_Backstage blueProp;
    private PIDController controller;

    public static double p=0.04, i=0, d=0.00001;//d=0.00001
    public static double f=0.002;

    public static double target;

    private final double ticks_in_degrees=288*(125/45.0)/360; /// gear ratio: 45/125=0.36

    private DcMotorEx elevator1, elevator2;
    private Robot robot;

    private Servo hangLeft,hangRight;
    private DcMotorEx viper,hang;



    private int tagID;

    private   final double ZERO_OFFSET = 70.0-3.85;
    private   double TargetPosInDegrees=70.0-3.85;
    private double armTarget=ZERO_OFFSET;
    private boolean eleDown=true;



    private ElapsedTime AUTO;





    public MathFunc mate;


    public FtcDashboard dashboard;
    private PIDController forward,strafe,turn;

    public double Pf=0.02d, If=0d, Df=0d;
    public double Ps=0.045d, Is=0d, Ds=0d;
    public  double Pt=0.02d, It=0.01d, Dt=0.00005d;


    private static double Targetf=1,Targets=1,Targett=1;

    public double val=0;



    public static double Distancef =8,Distances=6,Distancet=6;
    public static double DistancefAVION =18,DistancesAVION=15,DistancetAVION=15;

    public static double DistancefBACKDROP =8,DistancesBACKDROP=2,DistancetBACKDROP=5;


    public static double POWER_LiMIT=0.3;

    private int hope=0;

    @Override
    public void init() {

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        try {
            robot = new Robot(hardwareMap,telemetry);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        robot.camera.openBackCam();
        target=robot.arm.ZERO_OFFSET;

        dashboard=FtcDashboard.getInstance();

        forward=new PIDController(Pf,If,Df);
        forward.setPID(Pf,If,Df);

        strafe=new PIDController(Ps,Is,Ds);
        strafe.setPID(Ps,Is,Ds);

        turn=new PIDController(Pt,It,Dt);
        turn.setPID(Pt,It,Dt);

        telemetry=new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        hangLeft=hardwareMap.get(Servo.class,"hl");
        hangRight=hardwareMap.get(Servo.class,"hr");

        viper=hardwareMap.get(DcMotorEx.class,"v");
        viper.setDirection(DcMotorSimple.Direction.FORWARD);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hang=hardwareMap.get(DcMotorEx.class,"s");
        hang.setDirection(DcMotorSimple.Direction.REVERSE);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        timerr=new ElapsedTime();
        timerr.reset();

        setManualExposure(1,1);
    }

    @Override
    public void loop() {
        robot.wheels.reverseDirection();
        robot.arm.armTaskC();

        if(!once){
            robot.joint.setPosition(UP);
            robot.claw.setPosition(INTERMEDIARY);//CLAWINTERMEDIARY
            hangLeft.setPosition(0.35);
            hangRight.setPosition(0.65);
            once=true;
        }

        if(gamepad2.dpad_up){

            hangLeft.setPosition(0.7561111);
            hangRight.setPosition(0.24399999);

        }else if(gamepad2.dpad_down){

            hangLeft.setPosition(0.35);
            hangRight.setPosition(0.65);


        }

        if(gamepad2.b){
            if(!OKJoint && !PressJoint){
                robot.joint.setPosition(UP);
                OKJoint=true;

            }
            else if(OKJoint && !PressJoint){
                robot.joint.setPosition(DOWN);
                OKJoint=false;

            }
            PressJoint=true;

        }else {
            PressJoint=false;
        }

        if(gamepad2.dpad_right){
            robot.joint.setPosition(0.4);
            OKJoint=false;
        }

        if(gamepad2.dpad_left){
            robot.claw.setPosition(OPEN);//CLAWOPEN
            OKClaw=false;
        }

        if(gamepad2.a){
            if(!OKClaw && !PressClaw){
                robot.claw.setPosition(INTERMEDIARY);//CLAWINTERMEDIARY
                OKClaw=true;

            }
            else if(OKClaw && !PressClaw){
                robot.claw.setPosition(CLOSED);//CLAWCLOSED
                OKClaw=false;

            }
            PressClaw=true;
        }else{
            PressClaw=false;
        }

        if(gamepad2.right_stick_y<0){


            hang.setPower(-gamepad2.right_stick_y);

            hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        }else if(gamepad2.right_stick_y>0){

            hang.setPower(-gamepad2.right_stick_y);

            hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }

        else {

            hang.setPower(0);
            hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        if(gamepad2.left_stick_y>0){

            viper.setPower(-gamepad2.left_stick_y);
            viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }else if (gamepad2.left_stick_y<0){

            viper.setPower(-gamepad2.left_stick_y);
            viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }

        else{

            viper.setPower(0);
            viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        double drive =0;
        double strafe=0;
        double turn =0;

        boolean aprilk=false;

        if(robot.move.anyAprilTags(robot,robot.camera.atag)){

            //change color control hub to gold
            robot.move.showAprilTags(robot,robot.camera.atag);

        }else{

            // change color control hub to red
            telemetry.addLine("No April Tags :(");

        }

        if (gamepad1.dpad_left) {

            aprilk=true;

            tagID=4;

            Distancef=DistancefBACKDROP;
            Distances=DistancesBACKDROP;
            Distancet=DistancetBACKDROP;

           robot.wheels.setDirection();

            Go_to_April();



        }

        else if(gamepad1.dpad_up){

            aprilk=true;

            tagID=5;

            Distancef=DistancefBACKDROP;
            Distances=DistancesBACKDROP;
            Distancet=DistancetBACKDROP;

           robot.wheels.setDirection();

            Go_to_April();

        }

        else if(gamepad1.dpad_right){


            aprilk=true;

            tagID=6;

            Distancef=DistancefBACKDROP;
            Distances=DistancesBACKDROP;
            Distancet=DistancetBACKDROP;

          robot.wheels.setDirection();

            Go_to_April();
        }


        else if(gamepad1.dpad_down){


            aprilk=true;

            tagID=6;

            Distancef=DistancefAVION;
            Distances=DistancesAVION;
            Distancet=DistancetAVION;

            robot.wheels.setDirection();

            Go_to_April();
        }
        else{

            robot.wheels.reverseDirection();
            aprilk=false;

        }

        if(gamepad1.a){

            drive=-gamepad1.left_stick_y/1.25;

        }else if(gamepad1.b){

            strafe=-gamepad1.left_stick_x*1.1/1.25;

        }else {

            drive=-gamepad1.left_stick_y/1.25;
            strafe=-gamepad1.left_stick_x*1.1/1.25;
            turn=-gamepad1.right_stick_x/1.5;//2.5
        }

        if(aprilk==false){
            moveRobot(drive,strafe,turn);
        }

        //BRAT
        if(gamepad2.left_trigger>0){ // arm goes up

            if(timerr.seconds()>0.1 && armTarget<=270){
                armTarget+=gamepad2.left_trigger*10;
                timerr.reset();
            }
            robot.arm.setPositionC(armTarget,0.6);
            eleDown=false;

        }

        else if(gamepad2.left_bumper){ // arm goes up automatically

            armTarget=210;
            robot.arm.setPositionC(armTarget,0.6);
            eleDown=false;


        }

        else if(gamepad2.right_bumper){ // arm goes down automatically
            armTarget=ZERO_OFFSET;

            robot.arm.setPositionC(ZERO_OFFSET,0.45);
            eleDown=true;


        }

        else if(gamepad2.right_trigger>0){ // arm goes down

            if(timerr.seconds()>0.1 && armTarget-gamepad2.right_trigger*10>ZERO_OFFSET){
                armTarget-=gamepad2.right_trigger*10;
                timerr.reset();
            }

            robot.arm.setPositionC(armTarget,0.45);
            eleDown=true;


        }

        if(Math.abs(robot.arm.getPositionC()-ZERO_OFFSET)<5 && eleDown){

            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            robot.arm.setPower(0);
        }

        telemetry.update();




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
        robot.wheels.setPower(frontLeftPower,frontRightPower,backLeftPower,backRightPower);

    }


    private double ForwardPID(){


         Targetf=robot.move.returnRangeError(tagID,robot,robot.camera.atag);

        double power;


        if(Targetf==0){

            power=0;

        }else{
            double pid=forward.calculate(Targetf, Distancef);

            power = pid;

        }

        return  power;


        //  robot.wheels.setPower(-power,-power,-power,-power);

    }



    private double StrafePID(){

         Targets=robot.move.returnYawError(tagID,robot,robot.camera.atag);

        // target=matee.inchToTicksD(target);



        double power;


        if(Targets==0){

            power=0;

        }else{
            double pid=strafe.calculate(Targets, Distances);

            power = pid;

        }

        return power;


        // robot.wheels.setPower(-power,power,power,-power);

    }

    private double TurnPID(){


          Targett=robot.move.returnHeadingError(tagID,robot,robot.camera.atag);

        double power;


        if(Targett==0){

            power=0;

        }else{
            double pid=turn.calculate(Targett, Distancet);

            power = pid;

        }

        return  power;

        //  robot.wheels.setPower(-power,power,-power,power);

    }

    public void AprilPID(){

        double powerForward=ForwardPID();
        double powerStrafe=StrafePID();
        double powerTurn=TurnPID();

        double powerFrontLeft=-powerForward-powerStrafe-powerTurn;
        double powerFrontRight=-powerForward+powerStrafe+powerTurn;
        double powerBackLeft=-powerForward+powerStrafe-powerTurn;
        double powerBackRight=-powerForward-powerStrafe+powerTurn;

        double maxPower=MaxPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);

        if(maxPower>1){

            powerFrontLeft/=maxPower;
            powerFrontRight/=maxPower;
            powerBackLeft/=maxPower;
            powerBackRight/=maxPower;
        }

        maxPower=MaxPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);

        if(maxPower>POWER_LiMIT){

            double coeficient= maxPower/POWER_LiMIT;

            powerFrontLeft/=coeficient;
            powerFrontRight/=coeficient;
            powerBackLeft/=coeficient;
            powerBackRight/=coeficient;
        }


       robot.wheels.setPower(powerFrontLeft,powerFrontRight,powerBackLeft,powerBackRight);


    }

    public void Go_to_April(){


            AprilTagDetection detection=robot.move.returnAprilTAg(tagID,robot,robot.camera.atag);

            if(detection!=null){



                if(detection.ftcPose.range>=Distancef-2 && detection.ftcPose.range<=Distancef+7 && detection.ftcPose.bearing>=Distancet-7 && detection.ftcPose.bearing<=Distancet+7 && detection.ftcPose.yaw>=Distances-7 && detection.ftcPose.yaw<=Distances+7){

                    robot.wheels.setPower(0);
                    telemetry.addLine("AJUNS");


                }else{
                    AprilPID();
                    telemetry.addLine("PID YAY");

                }



                /*

                telemetry.addLine("Range: "+detection.ftcPose.range);
                telemetry.addLine("Bearing: "+detection.ftcPose.bearing);
                telemetry.addLine("Yaw: "+detection.ftcPose.yaw);


                 */
                 telemetry.update();


        }else{

                robot.wheels.setPower(0);

            }

    }


    private boolean   setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (robot.camera.vision == null) {
            return false;
        }

        // Wait for the camera to be open
        if (robot.camera.vision.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((robot.camera.vision.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!false)
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = robot.camera.vision.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(7, TimeUnit.MILLISECONDS);
            sleep(20);

            telemetry.addLine("Exposure Ceva: "+exposureControl);

            // Set Gain.
            GainControl gainControl = robot.camera.vision.getCameraControl(GainControl.class);

            telemetry.addLine("Gain Ceva: "+gainControl);

            if(gainControl!=null){
                boolean haide= gainControl.setGain(255);

            }
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }


}
