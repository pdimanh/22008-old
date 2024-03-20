package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class BlueAutoLeft extends LinearOpMode {


    private FirstVisionProcessor fvp;
    private VisionPortal visionPortal;
    private int locationofpixel = 1;
    //1 for left, 2 for middle, 3 for right
    private int useHueSatorVal = 2;
    //default is Saturation, 1 for hue 2 for saturation or 3 for value

    Servo bucketservo;
    Servo outtake;
    DcMotor LL, RL;
    Servo extension;
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        fvp = new FirstVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), fvp);

        bucketservo = hardwareMap.servo.get("bucketservo");
        outtake = hardwareMap.servo.get("outtake");
        LL = hardwareMap.dcMotor.get("LL");
        LL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RL = hardwareMap.dcMotor.get("RL");
        RL.setDirection(DcMotorSimple.Direction.REVERSE);
        RL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension = hardwareMap.servo.get("extension");



        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
//-y is the right of the robot
        //-x is backwards
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                .build();


        Trajectory drivetocenterprop = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-30, 0, Math.toRadians(0)))
                .build();
        Trajectory backupfromcenterprop = drive.trajectoryBuilder(drivetocenterprop.end())
                .lineToLinearHeading(new Pose2d(-23, 0, Math.toRadians(0)))
                .build();
        Trajectory turntobackdrop = drive.trajectoryBuilder(backupfromcenterprop.end())
                .lineToLinearHeading(new Pose2d(-23,-1, Math.toRadians(92)))
                .build();
        Trajectory drivetobackdrop = drive.trajectoryBuilder(turntobackdrop.end())
                .lineToLinearHeading(new Pose2d(-23, -43, Math.toRadians(92)))
                .build();

        Trajectory driveforward4rightside = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-12, 0, Math.toRadians(0)))
                .build();
        Trajectory drivewhilechangintangle4rightprop = drive.trajectoryBuilder(driveforward4rightside.end())
                .lineToLinearHeading(new Pose2d(-22 ,0 , Math.toRadians(-45)))
                .build();
        Trajectory drivetorightprop = drive.trajectoryBuilder(drivewhilechangintangle4rightprop.end())
                .lineToLinearHeading(new Pose2d(-26,8, Math.toRadians(-45)))
                .build();
        Trajectory backupfromrightprop = drive.trajectoryBuilder(drivetorightprop.end())
                .lineToLinearHeading(new Pose2d(-22,0, Math.toRadians(0)))
                .build();
        Trajectory rotatetobackdrop = drive.trajectoryBuilder(backupfromrightprop.end())
                .lineToLinearHeading(new Pose2d(-23, 0, Math.toRadians(92)))
                .build();
        Trajectory drivetorightbackdrop = drive.trajectoryBuilder(rotatetobackdrop.end())
                .lineToLinearHeading( new Pose2d(-32,-43, Math.toRadians(92)))
                .build();
        Trajectory drivetoleftprop = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-22,-6, Math.toRadians(18)))
                .build();
        Trajectory backupfromleftprop = drive.trajectoryBuilder(drivetoleftprop.end())
                .lineToLinearHeading(new Pose2d(-10,-4, Math.toRadians(0)))
                .build();
        Trajectory turntoleftbackdrop = drive.trajectoryBuilder(backupfromleftprop.end())
                .lineToLinearHeading(new Pose2d(-10,-3, Math.toRadians(92)))
                .build();
        Trajectory drivetoleftbackdrop = drive.trajectoryBuilder(turntoleftbackdrop.end())
                .lineToLinearHeading(new Pose2d(-19,-43, Math.toRadians(92)))
                .build();
        Trajectory backup1 = drive.trajectoryBuilder(drivetoleftbackdrop.end())
                .lineToLinearHeading(new Pose2d(-19, -38, Math.toRadians(92)))
                .build();
        Trajectory backup2 = drive.trajectoryBuilder(drivetobackdrop.end())
                .lineToLinearHeading(new Pose2d(-23, -38, Math.toRadians(92)))
                .build();
        Trajectory backup3 = drive.trajectoryBuilder(drivetorightbackdrop.end())
                .lineToLinearHeading(new Pose2d(-32, -38, Math.toRadians(92)))
                .build();
        Trajectory moveoutteway1 = drive.trajectoryBuilder(backup1.end())
                .lineToLinearHeading(new Pose2d(0, -38, Math.toRadians(92)))
                .build();
        Trajectory moveoutteway2 = drive.trajectoryBuilder(backup2.end())
                .lineToLinearHeading(new Pose2d(0, -38, Math.toRadians(92)))
                .build();
        Trajectory moveoutteway3 = drive.trajectoryBuilder(backup3.end())
                .lineToLinearHeading(new Pose2d(0, -38, Math.toRadians(92)))
                .build();



        while(opModeInInit()){

            displayHSV();
            determinePlacement();


        }

        waitForStart();

        if(isStopRequested()) return;
//do stuff here

        if(locationofpixel ==1){
            drive.followTrajectory(drivetoleftprop);
            sleep(10);
            drive.followTrajectory(backupfromleftprop);
            sleep(10);
            drive.followTrajectory(turntoleftbackdrop);
            sleep(10);
            drive.followTrajectory(drivetoleftbackdrop);
            sleep(5);
            sleep(5);
            scorepixel();
            sleep(10);
            drive.followTrajectory(backup1);
            drive.followTrajectory(moveoutteway1);
            sleep(10);
        }

        else if(locationofpixel == 2){
            drive.followTrajectory(drivetocenterprop);
            sleep(10);
            drive.followTrajectory(backupfromcenterprop);
            sleep(10);
            drive.followTrajectory(turntobackdrop);
            sleep(10);
            drive.followTrajectory(drivetobackdrop);
            sleep(5);
            sleep(5);
            scorepixel();
            sleep(10);
            drive.followTrajectory(backup2);
            drive.followTrajectory(moveoutteway2);


        }

        else{
            drive.followTrajectory(driveforward4rightside);
            sleep(10);
            drive.followTrajectory(drivewhilechangintangle4rightprop);
            sleep(10);
            drive.followTrajectory(drivetorightprop);
            sleep(10);
            drive.followTrajectory(backupfromrightprop);
            sleep(10);
            drive.followTrajectory(rotatetobackdrop);
            sleep(10);
            drive.followTrajectory(drivetorightbackdrop);
            sleep(5);
            sleep(5);
            scorepixel();
            sleep(10);
            drive.followTrajectory(backup3);
            drive.followTrajectory(moveoutteway3);
            sleep(10);
        }






    }
    private void scorepixel(){
       while( (RL.getCurrentPosition() < 300)){
           // LL.setPower(.3);
            RL.setPower(.3);
            sleep(50);
        }
           // LL.setPower(0);
            RL.setPower(0);
            sleep(500);
        extension.setPosition(.45);
        sleep(500);
        bucketservo.setPosition(.55);
        sleep(1000);
        outtake.setPosition(1);
        sleep(500);
        outtake.setPosition(.5);
        sleep(200);
        while( (RL.getCurrentPosition() < 350)){
           // LL.setPower(.3);
            RL.setPower(.3);
            sleep(50);
        }
        //LL.setPower(0);
        RL.setPower(0);
        bucketservo.setPosition(.8);
        sleep(100);
        extension.setPosition(0);
        sleep(200);

        while(RL.getCurrentPosition()>30){
          //  LL.setPower(-.3);
            RL.setPower(-.3);
            sleep(50);
        }
        // LL.setPower(0);
        RL.setPower(0);
        sleep(50);
    }

    public void displayHSV(){
        // telemetry.addData("Selection", fvp.getSelection());
        //telemetry.addData("left rect hue", fvp.leftrecthue);
        //telemetry.addData("left rect sat", fvp.leftrectsaturation);
        //telemetry.addData("left rect value", fvp.leftrectvalue);

        telemetry.addData("mid rect hue", fvp.midrecthue);
        telemetry.addData("mid rect sat", fvp.midrectsaturation);
        telemetry.addData("mid rect value", fvp.midrectvalue);

        telemetry.addData("right rect hue", fvp.rightrecthue);
        telemetry.addData("right rect sat", fvp.rightrectsaturation);
        telemetry.addData("right rect value", fvp.rightrectvalue);

        telemetry.addData("use 1 Hue, 2 Sat or 3 Value", useHueSatorVal);
        telemetry.addData("1 left, 2 mid, 3 right", locationofpixel);
        telemetry.update();

    }
    public void determinePlacement(){

        //1st person press left up or down to determine what method to use
        if(gamepad1.dpad_left){
            useHueSatorVal=1;
        }
        if(gamepad1.dpad_up){
            useHueSatorVal=2;
        }
        if(gamepad1.dpad_right){
            useHueSatorVal=3;
        }

        if(useHueSatorVal ==1){
            //using Hue
            if(Math.abs(fvp.midrecthue - fvp.rightrecthue)<30){

            } else {
                if (fvp.midrecthue>fvp.rightrecthue){
                    locationofpixel =2;
                } else{
                    locationofpixel =3;
                }

            }

        }


        if(useHueSatorVal ==2){
            //using Saturation
            if(Math.abs(fvp.midrectsaturation - fvp.rightrectsaturation)<30){
                locationofpixel =1;
            } else {
                if(fvp.midrectsaturation > fvp.rightrectsaturation){
                    locationofpixel =2;
                } else{
                    locationofpixel =3;
                }

            }

        }

        if(useHueSatorVal ==3){
            //using Value
            if(Math.abs(fvp.midrectvalue - fvp.rightrectvalue)<30){

            } else {
                if (fvp.midrectvalue>fvp.rightrectvalue){
                    locationofpixel =2;
                } else{
                    locationofpixel =3;
                }

            }


        }

    }



}
