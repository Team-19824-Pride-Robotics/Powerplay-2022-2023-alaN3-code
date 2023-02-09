package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(group = "drive")
@Config
public class test1 extends LinearOpMode {


    public void runOpMode() throws InterruptedException {


        DcMotor elevator;
        elevator = hardwareMap.get(DcMotor.class, "elevator");


        //elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        while (!isStopRequested()) {


            // telemetry
            telemetry.addData("Encoder elevator", elevator.getCurrentPosition());
            telemetry.update();

            if (gamepad1.y) {
                elevator.setPower(-.2);
            }
                if (gamepad1.x) {
                    elevator.setPower(.2);
                }
                else {
                    elevator.setPower(0);
                }
                }
            }
        }