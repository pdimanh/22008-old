package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TeleOp22008 extends OpMode {

private Servo gun;
private Servo gunaimer;

    private boolean MD = true;
    DcMotor FL, FR, BL, BR;
    DcMotor Climb;
    private IMU             imu         = null;
    Servo intake1;
    Servo outerintake;
    Servo release;
    DcMotor LL, RL;
    DcMotor intake2;
    Servo bucketservo;
    Servo outtake;
    Servo extension;
    Servo hangextension;
    private double liftlimit = 1195;
    private double downliftlimit = 0;
    private double bucketservoin = .8;
private double bucketservoout = .65;
private double extensionin = .1;
private double extensionout = .55;
public int ScoreState=0;
    private double drivescalar = 1;

    public void init() {
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        BL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.REVERSE);

        LL = hardwareMap.dcMotor.get("LL");
        LL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2 = hardwareMap.dcMotor.get("intake2");
        RL = hardwareMap.dcMotor.get("RL");
        RL.setDirection(DcMotorSimple.Direction.REVERSE);
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Climb = hardwareMap.dcMotor.get("Climb");
        Climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


/*
        RL.setDirection(DcMotorSimple.Direction.REVERSE);


        intake1 = hardwareMap.servo.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        bucketservo = hardwareMap.servo.get("bucketservo");
        outtake = hardwareMap.servo.get("outtake");

 */
        gun = hardwareMap.servo.get("gun");
        gun.setPosition(.5);

        gunaimer = hardwareMap.servo.get("gunaimer");
        gunaimer.setPosition(.5);
        outtake = hardwareMap.servo.get("outtake");
        extension = hardwareMap.servo.get("extension");

        extension.setPosition(0);
        hangextension = hardwareMap.servo.get("hangextension");
        hangextension.setPosition(1);
      //  intake1 = hardwareMap.servo.get("intake1");
       // outerintake = hardwareMap.servo.get("outerintake");
       // release = hardwareMap.servo.get("release");
      //  release.setPosition(.5);
        bucketservo = hardwareMap.servo.get("bucketservo");
        bucketservo.setPosition(bucketservoin);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot RevOrientation =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);

        imu.initialize(new IMU.Parameters(RevOrientation));


    }


    public void loop(){


      //  releaseServo();
        climbHandler();


        telemetryHandler();
        //fieldCentricSwitcher();
        intakeHandler();
       // lift();
       // intakehander();
      //  bucketservo();
        outtake();
        gunHandler();
       // extension();
        changescorestate();
        managescorestate();
      //  hangextension();




        if(gamepad1.y){
            imu.resetYaw();
        }
        /*if(gamepad1.left_trigger>0.5){
            overridelift();
        } else {
            lift();
        }*/
      /*  if(gamepad1.dpad_left){
        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

       */

        if(MD==true){
            robotCentricDrive();
        }
        if(MD==false){
            fieldCentricDrive();
        }
    }

private void gunHandler(){

        if(gamepad2.y){
            gun.setPosition(.7);
        } else {
            gun.setPosition(.5);
        }
        if(gamepad1.x){
            gunaimer.setPosition(0);
            hangextension.setPosition(.5);
        }




}



    private void robotCentricDrive(){
        double power = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;

        FL.setPower((power-turn-strafe)*drivescalar);
        FR.setPower((power+turn+strafe)*drivescalar);
        BL.setPower((power-turn+strafe)*drivescalar);
        BR.setPower((power+turn-strafe)*drivescalar);

        if(gamepad1.dpad_up){
            drivescalar = 1;
        }

        if(gamepad1.dpad_down){
            drivescalar = .5;
        }
    }

    private void climbHandler(){

        if(gamepad1.dpad_right){
            Climb.setPower(-1);
        } else if (gamepad1.dpad_left) {
            Climb.setPower(.8);
        } else {
            Climb.setPower(0);
        }
    }

    /* private void releaseServo(){
        if(release.getPosition() == .5){
            release.setPosition(1);

        }
        if (extension.getPosition()==0) {
            extension.setPosition(extensionin);
        }
        }

     */


    private void intakeHandler(){
        if(gamepad1.right_trigger>.5){
//intake into the robot
          //  intake1.setPosition(0);
           // outerintake.setPosition(.6);
            intake2.setPower(1);
        } else if (gamepad1.left_trigger>.5){

            //spit out of robot
           // intake1.setPosition(.9);
           // outerintake.setPosition(0);
            intake2.setPower(-.8);
        } else {
          //  intake1.setPosition(.5);
          //  outerintake.setPosition(.5);
            intake2.setPower(0);
        }

    }

    private void fieldCentricSwitcher(){

        if(gamepad1.left_bumper){
            MD=true;
        }
        if(gamepad1.right_bumper){
            MD=false;
        }

    }




    private void telemetryHandler(){

        double mycurrentangle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addData("angle",mycurrentangle);
        telemetry.addData("liftencoder", RL.getCurrentPosition());
        telemetry.addData("Robot Centric",MD);
        telemetry.addData("Drive Scalar", drivescalar);
        telemetry.update();
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)+Math.PI;
        // convert to polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        // rotate angle
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }


    private void fieldCentricDrive(){
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        driveFieldRelative(forward, right, rotate);
    }

    private void setPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));

        frontLeftPower /= maxSpeed;
        frontRightPower /= maxSpeed;
        backLeftPower /= maxSpeed;
        backRightPower /= maxSpeed;

        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right - rotate;
        double frontRightPower = forward - right + rotate;
        double backLeftPower = forward - right - rotate;
        double backRightPower = forward + right + rotate;

        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
/*
    private void intakehander(){
        if (gamepad1.right_trigger>0.5){
            intake1.setPosition(1);
            intake2.setPower(-1);
        }
        /*if(gamepad1.x){
            intake1.setPosition(.5);
            intake2.setPower(0);
        }
        else if(gamepad1.left_trigger>0.5){
            intake1.setPosition(-1);
            intake2.setPower(1);
        }
        else {
            intake1.setPosition(.5);
            intake2.setPower(0);
        }
    }
*/
    private void bucketservo(){
        //right to score, left to got back
        if (gamepad2.dpad_left){
            bucketservo.setPosition(bucketservoin);
        }
        if (gamepad2.dpad_right){
            bucketservo.setPosition(bucketservoout);
        }
    }
    private void extension(){
        if (extension.getPosition()==0){
            extension.setPosition(extensionin);
        }
        if (gamepad2.right_trigger>.5) {
            extension.setPosition(extensionout);
        }
        if (gamepad2.left_trigger>.5) {
            extension.setPosition(extensionin);
        }
    }
    private void hangextension(){
        if (gamepad1.right_bumper){
            hangextension.setPosition(.3);
        }
       else if(gamepad1.left_bumper){
            hangextension.setPosition(.7);
        }
       else{
           hangextension.setPosition(.5);
        }
    }

    private void outtake(){
        if (gamepad2.right_bumper || gamepad1.right_trigger>.5){
    outtake.setPosition(0);
        }
       else if (gamepad2.left_bumper || gamepad1.left_trigger>.5){
    outtake.setPosition(.6);
        }
       else{
           outtake.setPosition(.5);
        }
    }
    private void changescorestate(){
        if(gamepad2.dpad_up){
            ScoreState=1;
        }
        if(gamepad2.dpad_down){
            ScoreState=0;
        }
        if(gamepad2.dpad_left){
            ScoreState=2;
        }
    }

    private void managescorestate(){
        if(ScoreState==1){
            if(gamepad1.y){
                //LL.setPower(.2);
                RL.setPower(.2);
            }
            else {
            if(RL.getCurrentPosition() < 350){
                //LL.setPower(.3);
                RL.setPower(.3);
            }else{
               // LL.setPower(0);
                RL.setPower(0);
            }

            if(RL.getCurrentPosition() > 200){
                extension.setPosition(extensionout);
            }
            if(extension.getPosition()==extensionout){
                bucketservo.setPosition(bucketservoout);
            }
        }
        }
        if(ScoreState==0) {
            bucketservo.setPosition(bucketservoin);
            if (bucketservo.getPosition() == bucketservoin) {
                extension.setPosition(extensionin);
            }
            if (extension.getPosition() == extensionin) {
                if (RL.getCurrentPosition() > 10) {
                   // LL.setPower(-.1);
                    RL.setPower(-.1);
                } else {
                   // LL.setPower(0);
                    RL.setPower(0);
                }
            }
        }
            if(ScoreState==2){
                if(gamepad1.y){
                   // LL.setPower(.2);
                    RL.setPower(.2);
                }
                else {
                    if(RL.getCurrentPosition() < 250){
                       // LL.setPower(.3);
                        RL.setPower(.3);
                    }else{
                        //LL.setPower(0);
                        RL.setPower(0);
                    }

                    if(RL.getCurrentPosition() > 100){
                        extension.setPosition(extensionout);
                    }
                    if(extension.getPosition()==extensionout){
                        bucketservo.setPosition(bucketservoout);
                    }
                }
            }






        }
    
/*
    private void overridelift(){
        if(gamepad1.dpad_up){
            LL.setPower(.5);
            RL.setPower(.5);
        }
        else if(gamepad1.dpad_down){
            LL.setPower(-.5);
            RL.setPower(-.5);
    }
        else {
        LL.setPower(0);
        RL.setPower(0);
    }
    }

 */
    private  void lift(){

        if (gamepad1.dpad_up){
             // LL.setPower(.3);
           RL.setPower(.3);
        }
        else  if (gamepad1.dpad_down){
           // LL.setPower(-.3);
            RL.setPower(-.3);
        }
        else  {
           // LL.setPower(0);
           RL.setPower(0);
        }








/*
        if (gamepad2.dpad_up && LL.getCurrentPosition()<liftlimit){
            if(liftlimit - LL.getCurrentPosition() > 100){
               // LL.setPower(.3);
                RL.setPower(.3);
            } else {
                LL.setPower(.2);
                RL.setPower(.2);
            }

        } else if(gamepad2.dpad_down && LL.getCurrentPosition()>downliftlimit){
            if(LL.getCurrentPosition() - downliftlimit > 100){
               // LL.setPower(-.3);
                RL.setPower(-.3);
            } else {
               // LL.setPower(-.2);
                RL.setPower(-.2);
            }

        } else if(gamepad2.a){
            LL.setPower(.05);
            RL.setPower(.05);

        }else{
            LL.setPower(0);
            RL.setPower(0);
        }

 */




    }

}
