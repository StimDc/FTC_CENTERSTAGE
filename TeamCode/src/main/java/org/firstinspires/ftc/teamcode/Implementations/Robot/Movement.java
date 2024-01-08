package org.firstinspires.ftc.teamcode.Implementations.Robot;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.UniversalConsts.HEADING_ERROR_GAIN;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.UniversalConsts.LATERAL_ERROR;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.UniversalConsts.MAX_HEADING;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.UniversalConsts.MAX_RANGE;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.UniversalConsts.MAX_YAW;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.UniversalConsts.RANGE_ERROR_GAIN;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.UniversalConsts.ROTATE_ERROR;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.UniversalConsts.SLEEP;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.UniversalConsts.YAW_ERROR_GAIN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.Math.MathFunc;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class Movement {
   public  Wheels wheels;
   public IMU imu;
   public Telemetry telemetry;

   public static double heading =0,range =0,yaw = 0;
   public double HEADING;
    public void init(HardwareMap hardwareMap, Wheels wheels, IMU imu,Telemetry telemetry){
        this.wheels = wheels;
        this.imu = imu;
        this.telemetry = telemetry;
        wheels = new Wheels(hardwareMap);
        wheels.setDirection();
        wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void forward(int sign,double pow, double dist){
        //sign = MathFunc.valueToOne(sign);
        dist = (int)(MathFunc.inchToTicks(MathFunc.cmToInch(dist)));
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double x = dist - (int) MathFunc.inchToTicks(MathFunc.cmToInch(10 *pow / 0.6));

        while(wheels.isDistNotReached(x)){
            wheels.setPower(sign*pow);
        }
        while(wheels.isDistNotReached(dist)){

            if(dist>10){

                if(pow>0.01){
                    pow=pow-0.01;
                }
                if(pow<0.01){
                    pow=0.01;
                }

            }


            wheels.setPower(sign*pow);
        }
        wheels.setPower(0);
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(SLEEP);
    }

    public void lateral(int sign,double pow,double dist){
        //sign = MathFunc.valueToOne(sign);
        dist = dist*LATERAL_ERROR;

        int ticks = (int)((MathFunc.inchToTicks(MathFunc.cmToInch(dist))*(1/Math.sin(45))) *35/42);
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        wheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheels.setTargetPosition(ticks*sign,-ticks*sign,-ticks*sign,ticks*sign);
        wheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wheels.setPower(pow);
        wheels.waitMotors();
        wheels.setPower(0);
        wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(SLEEP);
    }

    public void generalMovement(double x, double y ,double yaw){
        double frontLeftPower = x-y-yaw;
        double frontRightPower = x+y+yaw;
        double backLeftPower = x+y-yaw;
        double backRightPower = x-y+yaw;

        double max = Math.max(Math.max(Math.abs(frontLeftPower),Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower),Math.abs(backRightPower)));
        if(max>1.0){
            frontLeftPower/=max;
            frontRightPower/=max;
            backRightPower/=max;
            backLeftPower/=max;
        }
        wheels.setPower(frontLeftPower,frontRightPower,backLeftPower,backRightPower);
        sleep(SLEEP);
    }

    public void rotate(int sign, double pow, double deg){
        //sign = MathFunc.valueToOne(sign);
        deg *=ROTATE_ERROR;
        double botHeading  = Math.abs(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))-Math.abs(HEADING);
        deg = Math.toRadians(deg);
        this.wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        while(Math.abs(botHeading)<Math.abs(deg)){
            this.wheels.setPower(pow*sign,-pow*sign,pow*sign,-pow*sign);
            botHeading = Math.abs(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS))-Math.abs(HEADING);

            telemetry.addLine("Heading" + this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }

        this.wheels.setPower(0);
        this.wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        HEADING = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addLine("Heading"+ this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.update();
        sleep(SLEEP);
    }


    public void Move_to_April(int idtag, Robot robot, AprilTagProcessor atag){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;
        boolean ok = false;
        int oldtarget=-1;

        boolean okHeading=false,okYaw=false;

        double rangeError = 100, headingError = 0, yawError = 0;

        while(!ok){
        //while (!ok && opModeIsActive() && !isStopRequested()) {

            targetFound=false;
            List<AprilTagDetection> currentDetections = atag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) && (detection.id == idtag)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

            if (targetFound) {

                    rangeError = desiredTag.ftcPose.range;


                if(okHeading==true){
                    headingError=0;
                }else{
                    headingError = desiredTag.ftcPose.bearing;
                }

                if(okYaw==true){
                    yawError=0;
                }else{
                    yawError = desiredTag.ftcPose.yaw; //1.7795
                }

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive = Range.clip(rangeError * RANGE_ERROR_GAIN, -MAX_RANGE,MAX_RANGE );
                double turn = Range.clip(headingError * HEADING_ERROR_GAIN, -MAX_HEADING, MAX_HEADING);
                double strafe = Range.clip(-yawError * YAW_ERROR_GAIN, -MAX_YAW, MAX_YAW);


                if(yawError>-2 && yawError<2){
                    okYaw=true;
                }
                if(headingError>-1 && headingError<1){
                    okHeading=true;
                }
                //moveRobot(-drive, -strafe, turn);

                if(okHeading==true && okYaw==true){
                    robot.move.generalMovement(-drive, 0, 0);
                }else{
                    robot.move.generalMovement(0, -strafe, turn);

                }
            //    robot.move.generalMovement(-drive, -strafe, turn);

            }else{
                robot.move.generalMovement(0,0,0);
            }
            if (rangeError < 25) {

                ok = true;
                robot.move.generalMovement(0,0,0);
            }
        }

        headingError=100;
        while(headingError>-1 && headingError<1){
            //while (!ok && opModeIsActive() && !isStopRequested()) {

            targetFound=false;
            List<AprilTagDetection> currentDetections = atag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) && (detection.id == idtag)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

            if (targetFound) {

                headingError = desiredTag.ftcPose.bearing;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double turn = Range.clip(headingError * HEADING_ERROR_GAIN, -MAX_HEADING, MAX_HEADING);

                //moveRobot(-drive, -strafe, turn);

                   robot.move.generalMovement(0, 0, turn);

            }else{
                robot.move.generalMovement(0,0,0);
            }
        }

    }

    public void Move_to_AprilAllAxes(int idtag, Robot robot, AprilTagProcessor atag){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;
        boolean ok = false;
        int oldtarget=-1;

        boolean okHeading=false,okYaw=false,okRange=false;

        double rangeError = 100, headingError = 0, yawError = 0;

        while(!ok){
            //while (!ok && opModeIsActive() && !isStopRequested()) {

            targetFound=false;
            List<AprilTagDetection> currentDetections = atag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) && (detection.id == idtag)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

            if (targetFound) {

                rangeError = desiredTag.ftcPose.range-12.75;



                headingError = desiredTag.ftcPose.bearing-5;


                yawError = desiredTag.ftcPose.yaw; //1.7795

               /* if(okRange==true){
                    rangeError=0;
                }

                if(okRange==true && okHeading==true){

                    headingError=0;

                }
                if(okRange==true && okYaw==true){

                    yawError=0;

                }

                */

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive = Range.clip(rangeError * RANGE_ERROR_GAIN, -MAX_RANGE,MAX_RANGE );
                double turn = Range.clip(headingError * HEADING_ERROR_GAIN, -MAX_HEADING, MAX_HEADING);
                double strafe = Range.clip(-yawError * YAW_ERROR_GAIN, -MAX_YAW, MAX_YAW);


                    robot.move.generalMovement(-drive, -strafe, turn);

            }else{
                robot.move.generalMovement(0,0,0);
            }
            if (rangeError>-range && rangeError < range && Math.abs(headingError) <heading &&Math.abs(yawError) <yaw)  {

              //  okRange=true;
                ok=true;
                robot.move.generalMovement(0,0,0);

            }
            /*
            else{
                okRange=false;
            }

            if(headingError>-1 && headingError<1){
                okHeading=true;
            }else{
                okHeading=false;
            }

            if(yawError>-1 && yawError<1){
                okYaw=true;
            }else{
                okYaw=false;
            }

            if(okYaw==true && okHeading==true && okRange==true){

                ok=true;
                robot.move.generalMovement(0,0,0);


            }

             */

        }



    }

}
