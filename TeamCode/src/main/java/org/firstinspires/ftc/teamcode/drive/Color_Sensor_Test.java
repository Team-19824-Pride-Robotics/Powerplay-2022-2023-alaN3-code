package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.ColorSensor;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@TeleOp(group = "drive")
@Config
public class Color_Sensor_Test extends LinearOpMode {

    ColorSensor color;

    RevBlinkinLedDriver lights;
    RevBlinkinLedDriver.BlinkinPattern pattern;





    public void runOpMode() throws InterruptedException {


        color = hardwareMap.get(ColorSensor.class, "Color");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");






        waitForStart();

        while (!isStopRequested()) {




            // telemetry
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.addData("luminosity", color.alpha());
            telemetry.addData("Combined color value", color.argb());
            telemetry.update();

        if (color.red() > 900 || color.blue() > 900) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                lights.setPattern(pattern);
            }

            else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                lights.setPattern(pattern);
            }
        }
    }
}