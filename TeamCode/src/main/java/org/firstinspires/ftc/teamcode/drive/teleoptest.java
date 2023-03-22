package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp(group = "drive")
@Config
public class teleoptest extends LinearOpMode {

    public static double elevator_strength = 1;
    public static double elevator_down_strength = 0.7;
    public static double speed = 1;
    public static double sr1o = 0.48;
    public static double sr1c = 0.7;
    public static double al = .06;
    public static double am = 0.40;
    public static double ar = .72;
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

    //pid
    public static int top = -1500;
    public static int mid = -1000;
    public static int low = -500;
    public static int pickup = -20;

    public static double Kp = .04;
    public static double Ki = 0;
    public static double Kd = .00015;

    public static double targetPos = 0;
    public static double integralSum = 0;
    public static double lastError = 0;
    public static double error = 0;
    public static double encoderPosition = 0;
    public static double derivative = 0;
    public static double power = 0;

    RevBlinkinLedDriver lights;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    private DcMotorEx elevator1;
    private DcMotorEx elevator2;




    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //drive.setPoseEstimate(PoseStorage.currentPose);

        Gyroscope imu;
        Servo servo1;
        Servo servo3;

        elevator1 = hardwareMap.get(DcMotorEx.class, "elevator1");
        elevator2 = hardwareMap.get(DcMotorEx.class, "elevator2");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");


        //reset encoder
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            telemetry.addData("claw1 pos",servo1.getPosition());
            telemetry.addData("arm pos",servo3.getPosition());
            telemetry.addData("Run time",getRuntime());
            telemetry.addData("pos", encoderPosition);
            telemetry.addData("target", targetPos);
            telemetry.addData("power", power);
            telemetry.update();


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
            if (gamepad2.y) {
                targetPos = top;
            }

            if (gamepad2.x) {
                targetPos = mid;
            }

            if (gamepad2.a) {
                targetPos = low;
            }

            if (gamepad2.b) {
                targetPos = pickup;
            }


            //reset the encoder in case it gets off track

            if(gamepad2.start) {

                elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }




//PID
            // Elapsed timer class from SDK, please use it, it's epic
            ElapsedTime timer = new ElapsedTime();

            // obtain the encoder position
            encoderPosition = elevator1.getCurrentPosition();
            // calculate the error
            error = targetPos - encoderPosition;

            // rate of change of the error
            derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            power = ((Kp * error) + (Ki * integralSum) + (Kd * derivative));

            elevator1.setPower(power);
            elevator2.setPower(power);

            lastError = error;

            // reset the timer for next time
            timer.reset();



        }
    }
}