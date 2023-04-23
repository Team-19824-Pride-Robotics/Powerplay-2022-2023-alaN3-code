package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name="romania_Left_Nala_3_Auto")

//@Disabled
public class romania_Left_Nala_3_Auto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public static double armMiddle = .40;
    public static int topCone = 300;
    public static int secondCone = 240;
    public static int thirdcone = 180;
    public static int lastcone = 100;
    public static double parkY = -25;
    public static double elevator_strength = 1;
    public static double elevator_down_strength = .7;
    public static double al = .06;
    public static double ar = .72;

    //junction
    public static int top = 2000;
    public static int mid = 1350;
    public static int low = 850;
    public static int pickup = 20;

    public static double turn = -90;
    // to first pole
    public static double x1 = 18.5;
    public static double y1 = 1;
    //move up to line up for pickup
    public static double x2 = 25;
    public static double y2 = -.5;

    public static double x3 = 28;
    public static double y3 = -1;



    //claw
    public static double sr1c = .68;
    public static double sr1o = .48;

    //april tag qr id
    int id = 3;

    //led
    int temp = 1;

    RevBlinkinLedDriver lights;
    RevBlinkinLedDriver.BlinkinPattern pattern;







    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT); //UPRIGHT
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        DcMotorEx elevator;
        Servo servo1;
        Servo servo3;

        elevator = hardwareMap.get(DcMotorEx.class, "elevator");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);


        //led
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        lights.setPattern(pattern);




        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections) {
                    telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
                    id = tag.id;
                }
            }
             else
                telemetry.addLine("Don't see tag :(");
            telemetry.update();
            sleep(20);


        }


        if (opModeIsActive()) {

            //led
            pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            lights.setPattern(pattern);

            //apriltag
            if (id == 0)
                parkY = -25;
            else if (id == 1)
                parkY = 0;
            else if (id == 2)
                parkY = 25;


            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                    //close the claw
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        servo1.setPosition(sr1c);
                    })

                    //drive to low junction
                    //.lineTo(new Vector2d(x1,y1))
                    .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(0)))

                    //move arm up, then swing it into position (while driving)
                    .UNSTABLE_addTemporalMarkerOffset(-.8, () -> {
                        elevator.setTargetPosition(low);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-.2, () -> {
                        servo3.setPosition(ar);
                    })

                    //time for the arm to stop swinging
                    .waitSeconds(.1)

                    //open claw and swing arm back to middle
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        servo1.setPosition(sr1o);
                    })

                    .lineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(0)))
                    .UNSTABLE_addTemporalMarkerOffset(-.5, () -> {
                        servo1.setPosition(sr1c);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.20, () -> {
                        servo3.setPosition(armMiddle);
                    })

                    .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(turn)))

                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(pickup);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_down_strength);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                        servo1.setPosition(sr1o);
                    })
                    .forward(parkY)

                    //time to score and then swing the arm back




                    .build();

            if (!isStopRequested()) {
                drive.followTrajectorySequence(trajSeq);
            }

//            PoseStorage.currentPose = drive.getPoseEstimate();





        }
    }


}