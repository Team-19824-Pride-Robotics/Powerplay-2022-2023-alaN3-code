package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(group = "drive")
@Config
public class yajietest extends LinearOpMode {


    public void runOpMode() throws InterruptedException {


        DcMotor elevator;
        elevator = hardwareMap.get(DcMotor.class, "LF");


        //elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (!isStopRequested()) {


            // telemetry
            telemetry.addData("Encoder elevator", elevator.getCurrentPosition());
            telemetry.update();

            if (gamepad1.y) {
                elevator.setPower(.5);
                elevator.setTargetPosition(-500);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (gamepad1.x) {
                elevator.setPower(.5);
                elevator.setTargetPosition(-20);
                elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


                }
            }
        }