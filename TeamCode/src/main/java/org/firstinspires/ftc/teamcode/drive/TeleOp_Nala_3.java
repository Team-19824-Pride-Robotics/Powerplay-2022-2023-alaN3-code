package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp(group = "drive")
@Config
public class TeleOp_Nala_3 extends LinearOpMode {

    public static double elevator_strength = 1;
    public static double elevator_down_strength = 0.7;
    public static double speed = 1;
    public static double sr1o = 0.48;
    public static double sr1c = 0.7;
    public static double al = .06;
    public static double am = 0.40;
    public static double ar = .72;
    public static double top = -2030;
    public static double mid = -1400;
    public static double low = -850;
    public static double ground = -100;
    public static double pickup = -30;
    public static double x1 = 35.91;
    public static double y1 = -28.22;
    public static double x2 = .754;
    public static double y2 = -23.276;
    public static double h1 = 180;
    public static double h2 = 180;
    public static double downToScore = 150;
    public static double bumpUpElevator = 170;
    public boolean ClawState = true;
    public static double pos = 0;
    int temp = 1;

    RevBlinkinLedDriver lights;
    RevBlinkinLedDriver.BlinkinPattern pattern;


    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //drive.setPoseEstimate(PoseStorage.currentPose);

        DcMotor elevator;
        Gyroscope imu;
        Servo servo1;
        Servo servo3;

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");


        //reset encoder
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //led
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        lights.setPattern(pattern);

        waitForStart();

        while (!isStopRequested()) {

            // telemetry
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("Encoder elevator", elevator.getCurrentPosition());
            telemetry.addData("claw1 pos",servo1.getPosition());
            telemetry.addData("arm pos",servo3.getPosition());
            telemetry.addData("Run time",getRuntime());
            telemetry.addData("temp",temp);
            telemetry.update();

            pos = elevator.getCurrentPosition();

            /*//////////////////////////
            DRIVER 1 CONTROLS START HERE
            *///////////////////////////

            if (gamepad1.right_bumper) {
                speed = 1;
            }
            if (gamepad1.left_bumper) {
                speed =.5;
            }
            else {
                speed = .75;
            }

            double driving = (-gamepad1.left_stick_y) * speed;
            double strafing = (-gamepad1.left_stick_x) * 0;
            double turning = (-gamepad1.right_stick_x) * speed;

            if (gamepad1.right_bumper)
                 driving = (-gamepad1.left_stick_y) * 1;

            if(gamepad1.left_trigger>0.3) {
                strafing = (gamepad1.left_trigger)*0.5;
            }
            if(gamepad1.right_trigger>0.3) {
                strafing = (-gamepad1.right_trigger)*0.5;
            }
            if(gamepad1.dpad_left) {
                strafing = -0.25;
            }
            if(gamepad1.dpad_right) {
                strafing = 0.25;
            }
            if(gamepad1.dpad_up) {
                driving = -0.25;
            }
            if(gamepad1.dpad_down) {
                driving = 0.25;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            (driving),
                            (strafing),
                            (turning)
                    )
            );

            drive.update();



            //////////////////////////////
            //Semi-autonomous routines start here
            //////////////////////////////


      /*      if (gamepad1.a) {
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToConstantHeading(new Vector2d(x2, y2), Math.toRadians(h2))
                        .addDisplacementMarker(1, () -> {
                            servo3.setPosition(am);
                            elevator.setTargetPosition((int) pickup);
                            elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            elevator.setPower(elevator_strength);

                        })
                        .build();
                drive.followTrajectorySequenceAsync(trajSeq);
            }

        if (gamepad1.y) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .splineToConstantHeading(new Vector2d(x1, y1), Math.toRadians(h1))
                    .addDisplacementMarker(.1, () -> {
                        servo3.setPosition(am);
                        elevator.setTargetPosition((int) top);
                        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        elevator.setPower(elevator_strength);

                    })
                    .build();
            drive.followTrajectorySequenceAsync(trajSeq);
        }

            if (gamepad1.y) {
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .back(36)
                        .build();
                drive.followTrajectorySequenceAsync(trajSeq);
            }


            if (gamepad1.a) {
                TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .forward(36)
                        .build();
                drive.followTrajectorySequenceAsync(trajSeq);
            }


*/
            /*//////////////////////////
            DRIVER 2 CONTROLS START HERE
            *///////////////////////////

            //open claw
            if(gamepad2.left_bumper /*&& ClawState == false*/) {
                servo1.setPosition(sr1o);
                //ClawState=true;
            }

            //close claw
            if(gamepad2.right_bumper /*&& ClawState == true*/) {
                servo1.setPosition(sr1c);
                //ClawState=false;
            }


            //arm to left
            if (gamepad2.dpad_right) {
                servo3.setPosition(al);
            }
            //arm to mid
            if (gamepad2.dpad_up) {
                servo3.setPosition(am);
            }
            //arm to right
            if (gamepad2.dpad_left) {
                servo3.setPosition(ar);
            }
            //elevator to ground terminal level
//            if (gamepad2.dpad_down) {
//                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                elevator.setTargetPosition((int) ground);
//                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevator.setPower(elevator_down_strength);
//            }
            //elevator to high junction level
            if (gamepad2.y) {
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevator.setTargetPosition((int) top);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }
            //elevator to middle junction level
            if (gamepad2.x) {
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevator.setTargetPosition((int) mid);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }
            //elevator to low junction level
            if (gamepad2.a) {
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevator.setTargetPosition((int) low);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }

            //turn off encoders and manually move elevator down
//            if (gamepad2.b) {
//                servo3.setPosition(am);
//                servo1.setPosition(sr1c);
//                elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                if (gamepad2.left_stick_y > 0.5) {
//                    elevator.setPower(gamepad2.left_stick_y * 0.5);
//                }
//            }

            // elevator to pickup level
            if (gamepad2.b) {
                servo3.setPosition(am);
                elevator.setTargetPosition((int) pickup);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_down_strength);
            }


            //to move elevator manually, press left stick button to drop elevator and
            //right stick button to raise it

            if (gamepad2.left_stick_button) {

                double score = elevator.getCurrentPosition() + downToScore;
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevator.setTargetPosition((int) score);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevator.setPower(elevator_strength);

            }
            if (gamepad2.right_stick_button) {

                double score = elevator.getCurrentPosition() - downToScore;
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevator.setTargetPosition((int) score);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevator.setPower(elevator_down_strength);

            }

            //reset the encoder in case it gets off track

            if(gamepad2.start) {

                elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }



//            if (gamepad2.b)  {
//                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                elevator.setTargetPosition((int) pos);
//                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                elevator.setPower(elevator_strength);
//            }

            //led control




        }
    }
}