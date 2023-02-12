package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(group = "drive")
@Config
public class encoder extends LinearOpMode {




    public void runOpMode() throws InterruptedException {
        DcMotor RF;
        DcMotor LF;
        DcMotor RB;



        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");






        waitForStart();

        while (!isStopRequested()) {


            telemetry.addData(" LF Encoder", LF.getCurrentPosition());
            telemetry.addData(" RB Encoder", RB.getCurrentPosition());
            telemetry.addData(" RF Encoder", RF.getCurrentPosition());
            telemetry.addData("Run time",getRuntime());
            telemetry.update();


        }
    }
}