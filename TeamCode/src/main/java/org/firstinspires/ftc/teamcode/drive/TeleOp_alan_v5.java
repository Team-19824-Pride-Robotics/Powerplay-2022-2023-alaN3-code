package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

@Config
@TeleOp(group = "drive")
public class TeleOp_alan_v5 extends LinearOpMode {

    private PIDController controller;

    public static double p = 0.04, i= 0, d= 0.001;
    public static double f = 0.01;

    public static int target = 0;

    public static double speed = 1;
    public static double sr1o = 0.48;
    public static double sr1c = 0.7;
    public static double al = .06;
    public static double am = 0.40;
    public static double ar = .72;
    public static int downToScore = 150;

    double elevatorPosition = 0;
    public static int top = -2000;
    public static int mid = -1500;
    public static int low = -1000;
    public static int pickup = -80;

    private final double ticks_in_degree = 384.5 / 180.0;

    private DcMotorEx elevator1;
    private DcMotorEx elevator2;

    //initialize our drive
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    //initialize our LEDs
    RevBlinkinLedDriver lights;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    Servo servo1;
    Servo servo3;

    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(p, i, d);
        elevator1 = hardwareMap.get(DcMotorEx.class, "elevator1");
        elevator2 = hardwareMap.get(DcMotorEx.class, "elevator2");

        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       // elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(PoseStorage.currentPose);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        lights.setPattern(pattern);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {

            elevatorPosition = elevator1.getCurrentPosition();

            /*//////////////////////////
            DRIVER 1 CONTROLS START HERE
            *///////////////////////////

            if (gamepad1.right_bumper) {
                speed = 1;
            }
            if (gamepad1.left_bumper) {
                speed = .5;
            } else {
                speed = .75;
            }

            double driving = (-gamepad1.left_stick_y) * speed;
            double strafing = (-gamepad1.left_stick_x) * 0;
            double turning = (-gamepad1.right_stick_x) * speed;

            if (gamepad1.right_bumper)
                driving = (-gamepad1.left_stick_y) * 1;

            if (gamepad1.left_trigger > 0.3) {
                strafing = (gamepad1.left_trigger) * 0.75;
            }
            if (gamepad1.right_trigger > 0.3) {
                strafing = (-gamepad1.right_trigger) * 0.75;
            }
            if (gamepad1.dpad_left) {
                strafing = -0.25;
            }
            if (gamepad1.dpad_right) {
                strafing = 0.25;
            }
            if (gamepad1.dpad_up) {
                driving = -0.25;
            }
            if (gamepad1.dpad_down) {
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
            if (gamepad2.left_bumper) {
                servo1.setPosition(sr1o);

            }

            //close claw
            if (gamepad2.right_bumper) {
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
            if (gamepad2.y) {
                target = top;
            }

            if (gamepad2.x) {
                target = mid;
            }

            if (gamepad2.a) {
                target = low;
            }

            if (gamepad2.b) {
                target = pickup;
            }

            if (gamepad2.right_stick_button) {

                target = elevator1.getCurrentPosition() - downToScore;
            }

            if (gamepad2.left_stick_button) {

                target = elevator1.getCurrentPosition() + downToScore;
            }

            controller.setPID(p, i, d);
            int elevPos = elevator1.getCurrentPosition();
            double pid = controller.calculate(elevPos, target);
            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

            double power = pid + ff;

            elevator1.setPower(power);
            elevator2.setPower(power);


            telemetry.addData("elevator Position: ", elevatorPosition);
            telemetry.addData("target position: ", target);
            telemetry.update();
        }
    }
}
