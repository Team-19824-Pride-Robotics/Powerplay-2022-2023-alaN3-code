package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(group = "drive")
@Config
public class Test_Teleop extends LinearOpMode {

    public static double elevator_strength = 1;
    public static double elevator_down_strength = .75;
    public static double speed = 1;
    public static double sr1o = 0.48;
    public static double sr1c = 0.67;
    public static double al = .06;
    public static double am = 0.40;
    public static double ar = .72;
    public static double top = -1950;
    public static double mid = -1400;
    public static double low = -850;
    public static double ground = -100;
    public static double pickup = -20;
    public static double downToScore = 150;
    public static double bumpUpElevator = 150;
    public static double elev  = 0;
    public static double servo = 0;





    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //drive.setPoseEstimate(PoseStorage.currentPose);

        DcMotor elevator;
        Servo servo1;
        Servo servo3;

        elevator = hardwareMap.get(DcMotor.class, "elevator");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo3 = hardwareMap.get(Servo.class, "servo3");


        //reset encoder
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //led


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
            telemetry.update();

            /*//////////////////////////
            DRIVER 1 CONTROLS START HERE
            *///////////////////////////


            if (gamepad1.left_bumper) {
                speed =1;
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



            if (gamepad1.right_bumper && gamepad1.y) {
                elevator.setTargetPosition((int) top);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
                if (elev > -1750) {
                    servo3.setPosition(al);
                }
            }
            if (gamepad1.right_bumper && gamepad1.x) {
                elevator.setTargetPosition((int) mid);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
                if (elev > -1200) {
                    servo3.setPosition(ar);
                }
            }
            if (gamepad1.right_bumper && gamepad1.a) {
                elevator.setTargetPosition((int) low);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
                elev = elevator.getCurrentPosition();
                if (elev > -800) {
                    servo3.setPosition(ar);
                }
            }
            if (gamepad1.right_bumper && gamepad1.b) {
                servo3.setPosition(am);
                servo = servo3.getPosition();
                if (servo > .38 || servo < .42) {
                    elevator.setTargetPosition((int) pickup);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(elevator_down_strength);
                }
            }
            if (gamepad1.right_bumper && gamepad1.dpad_right) {
                servo3.setPosition(al);
            }
            //arm to mid
            if (gamepad1.right_bumper && gamepad1.dpad_up) {
                servo3.setPosition(am);
            }
            //arm to right
            if (gamepad1.right_bumper && gamepad1.dpad_left) {
                servo3.setPosition(ar);
            }
            if (gamepad1.right_bumper && gamepad1.dpad_down) {
                elevator.setTargetPosition((int) ground);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }

            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                servo1.setPosition(sr1c);
            }
            if (gamepad1.right_bumper && gamepad1.right_stick_button) {
                servo1.setPosition(sr1o);
            }


            if (gamepad1.left_stick_button) {
                double score = elevator.getCurrentPosition() + downToScore;
                elevator.setTargetPosition((int) score);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }
            if (gamepad1.right_stick_button) {
                double raise = elevator.getCurrentPosition() - bumpUpElevator;
                elevator.setTargetPosition((int) raise);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elevator.setPower(elevator_strength);
            }



        }
    }
}