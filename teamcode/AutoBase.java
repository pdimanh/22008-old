package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous
public class AutoBase extends LinearOpMode {


    private FirstVisionProcessor fvp;
    private VisionPortal visionPortal;
private int locationofpixel = 1;
//1 for left, 2 for middle, 3 for right
    private int useHueSatorVal = 2;
    //default is Saturation, 1 for hue 2 for saturation or 3 for value

    Servo bucketservo;
    Servo outtake;
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        fvp = new FirstVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), fvp);

        bucketservo = hardwareMap.servo.get("bucketservo");
        outtake = hardwareMap.servo.get("outtake");


        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
//-y is the right of the robot
        //-x is backwards
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                .build();

/* blue side
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
                .lineToLinearHeading(new Pose2d(-25, -41, Math.toRadians(92)))
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
                .lineToLinearHeading( new Pose2d(-32,-41, Math.toRadians(92)))
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
                .lineToLinearHeading(new Pose2d(-19,-41, Math.toRadians(92)))
                .build();

 */
        Trajectory drivetocenterprop2 = drive.trajectoryBuilder(startPose).lineTo(new Vector2d(-28, 0)).build();
        Trajectory drivetocenterprop = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-28, 0, Math.toRadians(0)))
                .build();
        Trajectory backupfromcenterprop = drive.trajectoryBuilder(drivetocenterprop2.end())
                .lineToLinearHeading(new Pose2d(-23, 0, Math.toRadians(0)))
                .build();
        Trajectory turntobackdrop = drive.trajectoryBuilder(backupfromcenterprop.end())
                .lineToLinearHeading(new Pose2d(-23,-1, Math.toRadians(-90)))
                .build();
        Trajectory drivetobackdrop = drive.trajectoryBuilder(turntobackdrop.end())
                .lineToLinearHeading(new Pose2d(-27, 42, Math.toRadians(-90)))
                .build();

        Trajectory driveforward4rightside = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-12, 0, Math.toRadians(0)))
                .build();
        Trajectory drivewhilechangintangle4rightprop = drive.trajectoryBuilder(driveforward4rightside.end())
                .lineToLinearHeading(new Pose2d(-22 ,0 , Math.toRadians(18)))
                .build();
        Trajectory drivetorightprop = drive.trajectoryBuilder(drivewhilechangintangle4rightprop.end())
                .lineToLinearHeading(new Pose2d(-26,-5, Math.toRadians(18)))
                .build();
        Trajectory backupfromrightprop = drive.trajectoryBuilder(drivetorightprop.end())
                .lineToLinearHeading(new Pose2d(-22,0, Math.toRadians(0)))
                .build();
        Trajectory rotatetobackdrop = drive.trajectoryBuilder(backupfromrightprop.end())
                .lineToLinearHeading(new Pose2d(-23, 0, Math.toRadians(-90)))
                .build();
        Trajectory drivetorightbackdrop = drive.trajectoryBuilder(rotatetobackdrop.end())
                .lineToLinearHeading( new Pose2d(-32,42, Math.toRadians(-90)))
                .build();
        Trajectory drivetoleftprop = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-22,6, Math.toRadians(-45)))
                .build();
        Trajectory backupfromleftprop = drive.trajectoryBuilder(drivetoleftprop.end())
                .lineToLinearHeading(new Pose2d(-10,4, Math.toRadians(0)))
                .build();
        Trajectory turntoleftbackdrop = drive.trajectoryBuilder(backupfromleftprop.end())
                .lineToLinearHeading(new Pose2d(-10,3, Math.toRadians(-90)))
                .build();
        Trajectory drivetoleftbackdrop = drive.trajectoryBuilder(turntoleftbackdrop.end())
                .lineToLinearHeading(new Pose2d(-20,42, Math.toRadians(-90)))
                .build();


        while(opModeInInit()){

            displayHSV();
            determinePlacement();


        }

        waitForStart();

        if(isStopRequested()) return;
//do stuff here
        /* blue side
        if(locationofpixel ==1){
            drive.followTrajectory(drivetoleftprop);
            sleep(1000);
            drive.followTrajectory(backupfromleftprop);
            sleep(1000);
            drive.followTrajectory(turntoleftbackdrop);
            sleep(1000);
            drive.followTrajectory(drivetoleftbackdrop);
            sleep(1000);
        }

        else if(locationofpixel == 2){
            drive.followTrajectory(drivetocenterprop);
            sleep(1000);
            drive.followTrajectory(backupfromcenterprop);
            sleep(1000);
            drive.followTrajectory(turntobackdrop);
            sleep(1000);
            drive.followTrajectory(drivetobackdrop);
            sleep(1000);
        }

        else{
            drive.followTrajectory(driveforward4rightside);
            sleep(1000);
            drive.followTrajectory(drivewhilechangintangle4rightprop);
            sleep(1000);
            drive.followTrajectory(drivetorightprop);
            sleep(1000);
            drive.followTrajectory(backupfromrightprop);
            sleep(1000);
            drive.followTrajectory(rotatetobackdrop);
            sleep(1000);
            drive.followTrajectory(drivetorightbackdrop);
            sleep(1000);
        }

         */
        if(locationofpixel == 1){
            drive.followTrajectory(driveforward4rightside);
            sleep(1000);
            drive.followTrajectory(drivewhilechangintangle4rightprop);
            sleep(1000);
            drive.followTrajectory(drivetorightprop);
            sleep(1000);
            drive.followTrajectory(backupfromrightprop);
            sleep(1000);
            drive.followTrajectory(rotatetobackdrop);
            sleep(1000);
            drive.followTrajectory(drivetorightbackdrop);
            sleep(1000);
        }

        else if(locationofpixel == 2){
            drive.followTrajectory(drivetocenterprop);
            sleep(1000);
            drive.followTrajectory(backupfromcenterprop);
            sleep(1000);
            drive.followTrajectory(turntobackdrop);
            sleep(1000);
            drive.followTrajectory(drivetobackdrop);
            sleep(1000);;
        }

        else{ drive.followTrajectory(drivetoleftprop);
            sleep(1000);
            drive.followTrajectory(backupfromleftprop);
            sleep(1000);
            drive.followTrajectory(turntoleftbackdrop);
            sleep(1000);
            drive.followTrajectory(drivetoleftbackdrop);
            sleep(1000);

        }


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




