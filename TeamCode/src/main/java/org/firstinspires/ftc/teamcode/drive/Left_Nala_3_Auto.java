package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name="Left_Nala_3_Auto")

//@Disabled
public class Left_Nala_3_Auto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public static double armMiddle = 0.38;
    public static int topCone = -550;
    public static int secondCone = -400;
    public static int thirdcone = -250;
    public static double parkY = -13;
    public static double elevator_strength = 1;
    public static double al = .06;
    public static double ar = .73;

    //junction
    public static int top = -2900;
    public static int mid = -2150;
    public static int low = -1300;
    public static int pickup = -70;

    // to first pole
    public static double x1 = 38.2;
    public static double y1 = 3;
    //move up to line up for pickup
    public static double x2 = 49;
    public static double y2 = 2.5;
    //cone stack location
    public static double x3 = 49;
    public static double y3 = 23.5;
    //backup to score
    public static double x4 = 48.75;
    public static double y4 = -10.85;
    //score last cone on high
    public static double x5 = 48.75;
    public static double y5 = -10.85;

    //claw
    public static double sr1c = .67;
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
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        DcMotor elevator;
        Servo servo1;
        Servo servo3;

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //led
        pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
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
            pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            lights.setPattern(pattern);

            //apriltag
            if (id == 0)
                parkY = 31.8;
            else if (id == 1)
                parkY = 10;
            else if (id == 2)
                parkY = -13;

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)

                    //close the claw
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        servo1.setPosition(sr1c);
                    })

                    //drive to high junction
                    //.lineTo(new Vector2d(x1,y1))
                    .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(0)))
                    //move arm up, then swing it into position (while driving)
                    .UNSTABLE_addTemporalMarkerOffset(-1.75, () -> {
                        elevator.setTargetPosition(low);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                        servo3.setPosition(al);
                    })

                    //time for the arm to stop swinging
                    .waitSeconds(.25)

                    //open claw and swing arm back to middle
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        servo1.setPosition(sr1o);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                        servo3.setPosition(armMiddle);
                    })

                    //time to score and then swing the arm back
                    .waitSeconds(1)

                    //lower the elevator to "top cone" position
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(topCone);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);
                    })

                    //back up, turn, and then drive to cone stack
                    //.lineTo(new Vector2d(x2,y2))

                    .lineToLinearHeading(new Pose2d(x2, y2, Math.toRadians(0)))
                    .turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(90)))

                    //.lineTo(new Vector2d(x3,y3))
                   // .lineToLinearHeading(new Pose2d(x3, 2, Math.toRadians(0)))

                    //.splineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(90)), Math.toRadians(0))

                    //grab top cone and then raise the elevator up before backing away
                    .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                        servo1.setPosition(sr1c);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(top);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);
                    })

                    //time to grab the cone and raise elevator
                    .waitSeconds(0.5)

                    //drive to the high junction
                    //.lineTo(new Vector2d(x4,y4))
                    .lineToLinearHeading(new Pose2d(x4, y4, Math.toRadians(90)))

                    //swing the arm to the right while driving
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                        servo3.setPosition(ar);
                    })

                    //time for the arm to stop swinging
                    .waitSeconds(.25)

                    //open claw and swing arm back to middle
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        servo1.setPosition(sr1o);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.25, () -> {
                        servo3.setPosition(armMiddle);
                    })

                    //time to score and then swing the arm back
                    .waitSeconds(1.15)

                    //lower the elevator to "second cone" position
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(secondCone);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);
                    })

                    //drive back to the cone stack
                    .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(90)))

                    //.lineTo(new Vector2d(x3,y3))

                    //grab second cone and then raise the elevator up before backing away
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                        servo1.setPosition(sr1c);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(top);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);
                    })

                    //time to grab the cone and raise elevator
                    .waitSeconds(0.5)

                    //drive to the high junction
                    .lineToLinearHeading(new Pose2d(x4, y4, Math.toRadians(90)))
                    //.lineTo(new Vector2d(x4,y4))

                    //swing the arm to the right while driving
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                        servo3.setPosition(ar);
                    })

                    //time for the arm to stop swinging
                    .waitSeconds(.25)

                    //open claw and swing arm back to middle
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        servo1.setPosition(sr1o);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.25, () -> {
                        servo3.setPosition(armMiddle);
                    })
                    //time to score and then swing the arm back
                    .waitSeconds(1.15)

                    //lower the elevator to "second cone" position
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(thirdcone);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);
                    })

                    //drive back to the cone stack
                    .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(90)))

                    //.lineTo(new Vector2d(x3,y3))

                    //grab second cone and then raise the elevator up before backing away
                    .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                        servo1.setPosition(sr1c);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(top);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);
                    })

                    //time to grab the cone and raise elevator
                    .waitSeconds(0.5)

                    //drive to the high junction
                    .lineToLinearHeading(new Pose2d(x5, y5, Math.toRadians(90)))
                    //.lineTo(new Vector2d(x4,y4))

                    //swing the arm to the right while driving
                    .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                        servo3.setPosition(ar);
                    })

                    //time for the arm to stop swinging
                    .waitSeconds(.3)

                    //open claw and swing arm back to middle
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        servo1.setPosition(sr1o);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(.25, () -> {
                        servo3.setPosition(armMiddle);
                    })
                    //time to score and then swing the arm back
                    .waitSeconds(1.15)

                    //lower the elevator to pickup position
                    .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                        elevator.setTargetPosition(pickup);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);
                    })
                    .waitSeconds(1)
                    .UNSTABLE_addTemporalMarkerOffset(.3, () -> {
                        servo1.setPosition(sr1o);
                    })
                    //use the parkY variable to park in the correct zone
                    .forward(parkY)
                    .build();

            if (!isStopRequested()) {
                drive.followTrajectorySequence(trajSeq);
            }

            PoseStorage.currentPose = drive.getPoseEstimate();





        }
    }


}