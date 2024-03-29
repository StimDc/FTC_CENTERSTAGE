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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.checkerframework.common.value.qual.StringVal;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Implementations.Math.MathFunc;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
public class Movement {
    public FtcDashboard dashboard;

    public Wheels wheels;
    public IMU imu;
    public Telemetry telemetry;
    public double HEADING;

    public void init(HardwareMap hardwareMap, Wheels wheels, IMU imu, Telemetry telemetry) {
        this.wheels = wheels;
        this.imu = imu;
        this.telemetry = telemetry;
        this.wheels.setDirection();
        this.wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void forward(int sign, double pow, double dist) {
        //sign = MathFunc.valueToOne(sign);
        dist = (int) (MathFunc.inchToTicks(MathFunc.cmToInch(dist)));
        this.wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.wheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double x = dist - (int) MathFunc.inchToTicks(MathFunc.cmToInch(10 * pow / 0.6));

        while (this.wheels.isDistNotReached(x)) {
            this.wheels.setPower(sign * pow);
        }
        while (this.wheels.isDistNotReached(dist)) {

            if (dist > 10) {

                if (pow > 0.01) {
                    pow = pow - 0.01;
                }
                if (pow < 0.01) {
                    pow = 0.01;
                }

            }


            this.wheels.setPower(sign * pow);
        }
        this.wheels.setPower(0);
        this.wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(SLEEP);
    }

    public void lateral(int sign, double pow, double dist) {
        //sign = MathFunc.valueToOne(sign);
        dist = dist * LATERAL_ERROR;

        int ticks = (int) ((MathFunc.inchToTicks(MathFunc.cmToInch(dist)) * (1 / Math.sin(45))) * 35 / 42);
        this.wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.wheels.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.wheels.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.wheels.setTargetPosition(ticks * sign, -ticks * sign, -ticks * sign, ticks * sign);
        this.wheels.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.wheels.setPower(pow);
        this.wheels.waitMotors();
        this.wheels.setPower(0);
        this.wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(SLEEP);
    }

    public void generalMovement(double x, double y, double yaw) {
        double frontLeftPower = x - y - yaw;
        double frontRightPower = x + y + yaw;
        double backLeftPower = x + y - yaw;
        double backRightPower = x - y + yaw;

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
            backLeftPower /= max;
        }
        this.wheels.setPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        sleep(SLEEP);
    }

    public void rotate(int sign, double pow, double deg) {
        //sign = MathFunc.valueToOne(sign);
        deg *= ROTATE_ERROR;
        double botHeading = Math.abs(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)) - Math.abs(HEADING);
        deg = Math.toRadians(deg);
        this.wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        while (Math.abs(botHeading) < Math.abs(deg)) {
            this.wheels.setPower(pow * sign, -pow * sign, pow * sign, -pow * sign);
            botHeading = Math.abs(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)) - Math.abs(HEADING);

            this.telemetry.addLine("Heading" + this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            this.telemetry.update();
        }

        this.wheels.setPower(0);
        this.wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        HEADING = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        this.telemetry.addLine("Heading" + this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        this.telemetry.update();
        sleep(SLEEP);
    }


    public void Move_to_April(int idtag, Robot robot, AprilTagProcessor atag) {

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;
        boolean ok = false;
        int oldtarget = -1;

        boolean okHeading = false, okYaw = false;

        double rangeError = 100, headingError = 0, yawError = 0;

        while (!ok) {
            //while (!ok && opModeIsActive() && !isStopRequested()) {

            targetFound = false;
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


                if (okHeading == true) {
                    headingError = 0;
                } else {
                    headingError = desiredTag.ftcPose.bearing;
                }

                if (okYaw == true) {
                    yawError = 0;
                } else {
                    yawError = desiredTag.ftcPose.yaw; //1.7795
                }

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                double drive = Range.clip(rangeError * RANGE_ERROR_GAIN, -MAX_RANGE, MAX_RANGE);
                double turn = Range.clip(headingError * HEADING_ERROR_GAIN, -MAX_HEADING, MAX_HEADING);
                double strafe = Range.clip(-yawError * YAW_ERROR_GAIN, -MAX_YAW, MAX_YAW);


                if (yawError > -2 && yawError < 2) {
                    okYaw = true;
                }
                if (headingError > -1 && headingError < 1) {
                    okHeading = true;
                }
                //moveRobot(-drive, -strafe, turn);

                if (okHeading == true && okYaw == true) {
                    this.generalMovement(-drive, 0, 0);
                } else {
                    this.generalMovement(0, -strafe, turn);

                }
                //    this.generalMovement(-drive, -strafe, turn);

            } else {
                this.generalMovement(0, 0, 0);
            }
            if (rangeError < 25) {

                ok = true;
                this.generalMovement(0, 0, 0);
            }
        }

        headingError = 100;
        while (headingError > -1 && headingError < 1) {
            //while (!ok && opModeIsActive() && !isStopRequested()) {

            targetFound = false;
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

                this.generalMovement(0, 0, turn);

            } else {
                this.generalMovement(0, 0, 0);
            }
        }

    }

    public void Move_to_AprilAllAxes(int idtag, Robot robot, AprilTagProcessor atag) {

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;
        boolean ok = false,okplease=false;
        int oldtarget = -1;

        int help=0, ceva=1;

        boolean okHeading = false, okYaw = false, okRange = false;

        double rangeErrorSpeed=MAX_RANGE, headingErrorSpeed=MAX_HEADING, yawErrorSpeed=MAX_HEADING;

        double rangeError = 100, headingError = 0, yawError = 0;

        int error=0;

        while (!ok) {
            //while (!ok && opModeIsActive() && !isStopRequested()) {

            targetFound = false;
            List<AprilTagDetection> currentDetections = atag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) && (detection.id == idtag)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

            if (targetFound) {

                rangeError = desiredTag.ftcPose.range - 9.5;// pentru red center backdrop: 10 sau 10.5 pentru red right backdrop: 9


                headingError = desiredTag.ftcPose.bearing+2.5;//5


                yawError = desiredTag.ftcPose.yaw; //1.7795, 2.3

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
                /*
                double drive = Range.clip(rangeError * RANGE_ERROR_GAIN, -MAX_RANGE, MAX_RANGE);
                double turn = Range.clip(headingError * HEADING_ERROR_GAIN, -MAX_HEADING, MAX_HEADING);
                double strafe = Range.clip(-yawError * YAW_ERROR_GAIN, -MAX_YAW, MAX_YAW);


                 */
                double drive = Range.clip(rangeError * RANGE_ERROR_GAIN, -rangeErrorSpeed, rangeErrorSpeed);
                double turn = Range.clip(headingError * HEADING_ERROR_GAIN, -headingErrorSpeed, headingErrorSpeed);
                double strafe = Range.clip(-yawError * YAW_ERROR_GAIN, -yawErrorSpeed, yawErrorSpeed);

                help=0;

                this.generalMovement(-drive, -strafe, turn);


            } else {

                /*
                help=help+ceva;

                if(help==100000){

                    help=0;
                    ceva=ceva*(-1);
                    this.generalMovement(0, 0, 100);


                }else if(help==-100000){

                    help=0;
                    ceva=ceva*(-1);
                    this.generalMovement(0, 0, -100);

                }

                 */

                error++;

                this.generalMovement(0, 0, 0);
            }

            if(error>15){
                ok=true;
            }

            if (rangeError > -10 && rangeError < 10) {

                //  okRange=true;
                okplease = true;
                headingErrorSpeed=0.1;
                yawErrorSpeed=0.1;
                rangeErrorSpeed=0.15;

            }

            if(okplease==true && rangeError > -2 && rangeError < 2  && headingError>-5 && headingError<5){

                ok=true;

                this.generalMovement(0, 0, 0);


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
                this.generalMovement(0,0,0);


            }

             */

            this.telemetry.addLine("Range Error "+ rangeError);
            this.telemetry.addLine("Heading Error "+headingError);
            this.telemetry.addLine("Yaw Error "+yawError);
            this.telemetry.update();

        }


    }


    public void CorrectingTurn(int idtag, Robot robot, AprilTagProcessor atag) {

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;
        boolean ok = false;
        int oldtarget = -1;

        int help=0, ceva=1;

        boolean okHeading = false, okYaw = false, okRange = false;

        double rangeError = 100, headingError = 0, yawError = 0;

        while (!ok) {
            //while (!ok && opModeIsActive() && !isStopRequested()) {

            targetFound = false;
            List<AprilTagDetection> currentDetections = atag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) && (detection.id == idtag)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

            if (targetFound) {

                headingError = desiredTag.ftcPose.bearing;//5

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
                double turn = Range.clip(headingError * HEADING_ERROR_GAIN, -MAX_HEADING, MAX_HEADING);

                help=0;

                this.generalMovement(0, 0, turn);


            } else {

                /*
                help=help+ceva;

                if(help==100000){

                    help=0;
                    ceva=ceva*(-1);
                    this.generalMovement(0, 0, 100);


                }else if(help==-100000){

                    help=0;
                    ceva=ceva*(-1);
                    this.generalMovement(0, 0, -100);

                }

                 */

                this.generalMovement(0, 0, 0);
            }
            if (headingError > -5 && headingError < 5) {

                //  okRange=true;
                ok = true;
                this.generalMovement(0, 0, 0);

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
                this.generalMovement(0,0,0);


            }

             */

        }


    }



    boolean STATE=false;

    public void Move_to_AprilAllAxes_StateMachine(int idtag, Robot robot, AprilTagProcessor atag) {

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;
        boolean ok = false;
        int oldtarget = -1;

        boolean okHeading = false, okYaw = false, okRange = false;

        double rangeError = 100, headingError = 0, yawError = 0;


        //while (!ok && opModeIsActive() && !isStopRequested()) {

        targetFound = false;
        List<AprilTagDetection> currentDetections = atag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == idtag)) {
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
        }

        if (targetFound) {

            rangeError = desiredTag.ftcPose.range - 10;


            headingError = desiredTag.ftcPose.bearing - 5;


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
            double drive = Range.clip(rangeError * RANGE_ERROR_GAIN, -MAX_RANGE, MAX_RANGE);
            double turn = Range.clip(headingError * HEADING_ERROR_GAIN, -MAX_HEADING, MAX_HEADING);
            double strafe = Range.clip(-yawError * YAW_ERROR_GAIN, -MAX_YAW, MAX_YAW);


            this.generalMovement(-drive, -strafe, turn);

        } else {
            this.generalMovement(0, 0, 0);
        }
        if (rangeError > -2 && rangeError < 2) {

            //  okRange=true;
            STATE=true;
            this.generalMovement(0, 0, 0);

        }

        if(STATE){

            this.generalMovement(0, 0, 0);


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
                this.generalMovement(0,0,0);


            }

             */



    }

    public boolean getArmState(){

        return STATE;
    }

    public double returnHeadingError(int idtag,Robot robot, AprilTagProcessor atag){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;

        //while (!ok && opModeIsActive() && !isStopRequested()) {

        targetFound = false;
        List<AprilTagDetection> currentDetections = atag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == idtag || idtag==0)) {
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
        }

        if(targetFound) {

            return desiredTag.ftcPose.bearing;


        }else{

            return 0;

        }

    }

    public double returnRangeError(int idtag,Robot robot, AprilTagProcessor atag){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;

        //while (!ok && opModeIsActive() && !isStopRequested()) {

        List<AprilTagDetection> currentDetections = atag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == idtag || idtag==0)) {
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
        }

        if(targetFound) {

            return desiredTag.ftcPose.range;


        }else{

            return 0;

        }

    }

    public double returnYawError(int idtag,Robot robot, AprilTagProcessor atag){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;

        //while (!ok && opModeIsActive() && !isStopRequested()) {

        targetFound = false;
        List<AprilTagDetection> currentDetections = atag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == idtag || idtag==0)) {
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
        }

        if(targetFound) {

            return desiredTag.ftcPose.yaw;


        }else{

            return 0;

        }

    }


    public double returnXPoseError(int idtag,Robot robot, AprilTagProcessor atag){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;

        //while (!ok && opModeIsActive() && !isStopRequested()) {

        targetFound = false;
        List<AprilTagDetection> currentDetections = atag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == idtag || idtag==0)) {
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
        }

        if(targetFound) {

            return desiredTag.ftcPose.x;


        }else{

            return 0;

        }

    }

    public void showAllPose(int idtag,Robot robot, AprilTagProcessor atag){

        dashboard=FtcDashboard.getInstance();
        telemetry=new MultipleTelemetry(telemetry, dashboard.getTelemetry());



        AprilTagDetection desiredTag = null;
        boolean targetFound = false;

        //while (!ok && opModeIsActive() && !isStopRequested()) {

        targetFound = false;
        List<AprilTagDetection> currentDetections = atag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == idtag || idtag==0)) {
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
        }

        if(targetFound) {

            telemetry.addLine("XPose: "+desiredTag.ftcPose.x);
            telemetry.addLine("YPose: "+desiredTag.ftcPose.y);
            telemetry.addLine("ZPose: "+desiredTag.ftcPose.z);

            telemetry.addLine("Range: "+desiredTag.ftcPose.range);
            telemetry.addLine("Bearing: "+desiredTag.ftcPose.bearing);
            telemetry.addLine("Elevation: "+desiredTag.ftcPose.elevation);

            telemetry.addLine("Yaw: "+desiredTag.ftcPose.yaw);
            telemetry.addLine("Pitch: "+desiredTag.ftcPose.pitch);
            telemetry.addLine("Roll: "+desiredTag.ftcPose.roll);
            telemetry.update();



        }

    }

    public AprilTagDetection returnAprilTAg(int idtag, Robot robot, AprilTagProcessor atag){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;

        //while (!ok && opModeIsActive() && !isStopRequested()) {

        targetFound = false;
        List<AprilTagDetection> currentDetections = atag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == idtag || idtag==0 || detection.id-3==idtag || detection.id+3==idtag)) {
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            }
        }

        if(targetFound) {

            return desiredTag;


        }else{

            return null;

        }

    }


    public void showAprilTags(Robot robot, AprilTagProcessor atag){


        String s;
        boolean ok=false;
        //while (!ok && opModeIsActive() && !isStopRequested()) {

        int[] fr={0,0,0,0,0,0,0,0,0,0,0};

        List<AprilTagDetection> currentDetections = atag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                fr[detection.id]=1;

            }
        }

        for(int i=1;i<=6;i++){

            if(fr[i]!=0){

                ok=true;

                s= String.valueOf(i);

                telemetry.addLine(s);

            }

        }
        if(ok==false){

            telemetry.addLine("Nope Atags :/");

        }


    }


    public boolean anyAprilTags (Robot robot, AprilTagProcessor atag){

        List<AprilTagDetection> currentDetections = atag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {


                return true;
            }
        }

        return false;

    }




}