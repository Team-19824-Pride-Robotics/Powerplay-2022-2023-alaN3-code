package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(group = "drive")
@Config
public class TeleOp_Nala_3 extends LinearOpMode {

    public static double elevator_strength = 2000;
    public static double elevator_down_strength = 800;
    public static double speed = 1;
    public static double sr1o = 0.48;
    public static double sr1c = 0.7;
    public static double al = .06;
    public static double am = 0.40;
    public static double ar = .72;
    public static double top = -2030;
    public static double mid = -1400;
    public static double low = -850;
    public static double pickup = -30;
    public static double downToScore = 150;
    public static double pos = 0;
    int temp = 1;

    RevBlinkinLedDriver lights;
    RevBlinkinLedDriver.BlinkinPattern pattern;


    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //drive.setPoseEstimate(PoseStorage.currentPose);

        DcMotorEx elevator;
        Servo servo1;
        Servo servo3;

        elevator = hardwareMap.get(DcMotorEx.class, "elevator");
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
//            telemetry.addData("x", poseEstimate.getX());
//            telemetry.addData("y", poseEstimate.getY());
//            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("Elevator position: ", elevator.getCurrentPosition());
//            telemetry.addData("claw1 pos",servo1.getPosition());
//            telemetry.addData("arm pos",servo3.getPosition());
//            telemetry.addData("Run time",getRuntime());
//            telemetry.addData("temp",temp);
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

            /*//////////////////////////
            DRIVER 2 CONTROLS START HERE
            *///////////////////////////

            //open claw
            if(gamepad2.left_bumper) {
                servo1.setPosition(sr1o);
            }

            //close claw
            if(gamepad2.right_bumper) {
                servo1.setPosition(sr1c);
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

            //elevator to high junction level
            if (gamepad2.y) {
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevator.setTargetPosition((int) top);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setVelocity(elevator_strength);
            }

            //elevator to middle junction level
            if (gamepad2.x) {

                pos = elevator.getCurrentPosition();
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevator.setTargetPosition((int) mid);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(pos < mid) {
                    elevator.setVelocity(elevator_down_strength);
                }
                else {
                    elevator.setVelocity(elevator_strength);
                }
            }

            //elevator to low junction level
            if (gamepad2.a) {

                pos = elevator.getCurrentPosition();
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevator.setTargetPosition((int) low);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(pos < low) {
                    elevator.setVelocity(elevator_down_strength);
                }
                else {
                    elevator.setVelocity(elevator_strength);
                }
            }

            // elevator to pickup level
            if (gamepad2.b) {
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                servo3.setPosition(am);
                elevator.setTargetPosition((int) pickup);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setVelocity(elevator_down_strength);
            }

            //to move elevator manually, press left stick button to drop elevator and
            //right stick button to raise it

            if (gamepad2.left_stick_button) {

                double score = elevator.getCurrentPosition() + downToScore;
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevator.setTargetPosition((int) score);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setVelocity(elevator_strength);

            }
            if (gamepad2.right_stick_button) {

                double score = elevator.getCurrentPosition() - downToScore;
                elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                elevator.setTargetPosition((int) score);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setVelocity(elevator_down_strength);

            }

            //reset the encoder in case it gets off track

            if(gamepad2.start) {

                elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }
        }
    }
}