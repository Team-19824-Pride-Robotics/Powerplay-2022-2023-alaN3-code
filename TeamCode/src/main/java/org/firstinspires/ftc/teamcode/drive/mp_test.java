package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class mp_test extends OpMode {

    public static double start = 0;
    public static double target = 1000;
    public static double power = 0;
    public static double distance = 0;
    public static double seg = 0;
    public static double current = 0;

    private DcMotorEx elevator1;
    private DcMotorEx elevator2;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elevator1 = hardwareMap.get(DcMotorEx.class, "elevator1");
        elevator2 = hardwareMap.get(DcMotorEx.class, "elevator2");
        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

    current = elevator1.getCurrentPosition();
    seg = distance / 10;

        if (current >= start && current < start + seg) {
            power = .3;
        }

        else if (current >= start + seg && current < start + (seg * 2)) {
            power = .6;
        }

        else if (current >= start + (seg * 2) && current < start + (seg * 8)) {
            power = 1;
        }

        else if (current >= start + (seg * 8) && current < start + (seg * 9)){
            power = .6;
        }

        else if (current >= start + (seg * 9) && current < start + (seg * 10)) {
            power = .3;
        }

        else {
            power = .15;
        }



        elevator1.setPower(power);
        elevator2.setPower(power);


        telemetry.addData("pos", current);
        telemetry.addData("target", target);
        telemetry.addData("power", power);
        telemetry.addData("start", start);
        telemetry.addData("power", seg);
        telemetry.addData("motorpower1", elevator1.getPower());
        telemetry.addData("motorpower2", elevator2.getPower());

        telemetry.update();
    }


}
