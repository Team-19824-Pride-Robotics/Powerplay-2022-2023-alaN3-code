package org.firstinspires.ftc.teamcode.drive;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDF_Test extends OpMode {

    private PIDController controller;

    public static double p = 0, i= 0, d= 0;
    public static double f = 0;

    public static int target = 0;

    private final double tick_in_degree = 312 / 180.0;

    private DcMotorEx elevator1;
    private DcMotorEx elevator2;


    @Override
    public void init() {
        controller= new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elevator1 = hardwareMap.get(DcMotorEx.class, "elevator");
        elevator2 = hardwareMap.get(DcMotorEx.class, "elevator");
        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int elevPos = elevator1.getCurrentPosition();
        double pid = controller.calculate(elevPos, target);
        double ff =Math.cos(Math.toRadians(target / tick_in_degree)) * f;

        double power = pid + ff;

        elevator1.setPower(power);
        elevator2.setPower(power);


        telemetry.addData("pos", elevPos);
        telemetry.addData("target", target);
        telemetry.update();
    }


}
