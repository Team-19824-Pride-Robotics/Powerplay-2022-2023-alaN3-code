/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




@TeleOp(group="Drive")
@Config
public class PID_Test extends LinearOpMode {

    public static double Kp = .02;
    public static double Ki = 0;
    public static double Kd = .0003;

    public static double targetPos = 0;
    public static double integralSum = 0;
    public static double lastError = 0;
    public static double error = 0;
    public static double encoderPosition = 0;
    public static double derivative = 0;
    public static double power = 0;

    public static int top = -1500;
    public static int mid = -1000;
    public static int low = -500;
    public static int pickup = -20;

    private DcMotorEx elevator1;
    private DcMotorEx elevator2;



    public void runOpMode() throws InterruptedException {

        elevator1 = hardwareMap.get(DcMotorEx.class, "elevator1");
        elevator2 = hardwareMap.get(DcMotorEx.class, "elevator2");
        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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

            if (gamepad1.y) {
                targetPos = top;
            }

            if (gamepad1.x) {
                targetPos = mid;
            }

            if (gamepad1.a) {
                targetPos = low;
            }

            if (gamepad1.b) {
                targetPos = pickup;
            }


            telemetry.addData("pos", encoderPosition);
            telemetry.addData("target", targetPos);
            telemetry.addData("power", power);
            telemetry.addData("integralSum", integralSum);
            telemetry.addData("lastError", lastError);
            telemetry.addData("error", error);
            telemetry.addData("derivative", derivative);
            telemetry.update();


        }
    }
}
