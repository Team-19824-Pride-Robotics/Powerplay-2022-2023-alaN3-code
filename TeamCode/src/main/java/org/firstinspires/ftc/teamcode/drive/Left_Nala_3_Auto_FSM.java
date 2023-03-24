package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name="Left_Nala_3_Auto_FSM")

//@Disabled
public class Left_Nala_3_Auto_FSM extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TURN_1,         // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        WAIT_1,         // Then we're gonna wait a second
        TURN_2,         // Finally, we're gonna turn again
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));


    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public static double armMiddle = .40;
    public static int topCone = -300;
    public static int secondCone = -240;
    public static int thirdcone = -180;
    public static double parkY = -17;
    public static double elevator_strength = 1;
    public static double elevator_down_strength = .7;
    public static double al = .06;
    public static double ar = .72;

    private DcMotorEx elevator1;
    private DcMotorEx elevator2;

    public static int top = -1500;
    public static int mid = -1000;
    public static int low = -500;
    public static int pickup = -20;

    public static double Kp = .004;
    public static double Ki = 0;
    public static double Kd = .00009;

    public static double targetPos = 0;
    public static double integralSum = 0;
    public static double lastError = 0;
    public static double error = 0;
    public static double encoderPosition = 0;
    public static double derivative = 0;
    public static double power = 0;

    public static double turn = 90;

    // to first pole
    public static double x1 = 43;
    public static double y1 = -.5;
    //move up to line up for pickup
    public static double x2 = 50;
    public static double y2 = -.5;
    //cone stack location
    public static double x3 = 51.5;
    public static double y3 = 25.5;
    //backup to score
    public static double x4 = 51.4;
    public static double y4 = -8.2;
    //score last cone on high
    public static double x5 = 51.4;
    public static double y5 = -8.2;
    //score second cone on high
    public static double x7 = 51.3;
    public static double y7 = -8;
    //push cone out the way
    public static double x6 = 60;
    public static double y6 = -.5;

    //claw
    public static double sr1c = .68;
    public static double sr1o = .48;

    //april tag qr id
    int id = 3;

    //led
    int temp = 1;

    //public double targetPos;

    RevBlinkinLedDriver lights;
    RevBlinkinLedDriver.BlinkinPattern pattern;



    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT); //UPRIGHT
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //Initialize our drive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Initialize our lift
        Lift lift = new Lift(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        DcMotorEx elevator1;
        DcMotorEx elevator2;
        Servo servo1;
        Servo servo3;

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //led
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        lights.setPattern(pattern);

        // Let's define our trajectories
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(x1, y1), Math.toRadians(0))
                .addTemporalMarker(0, () -> {
                    servo1.setPosition(sr1c);
                    targetPos = mid;
                })
                .addTemporalMarker(1.5, () -> {
                    servo3.setPosition(al);
                })
                .addTemporalMarker(2, () -> {
                    servo1.setPosition(sr1o);
                })
                .build();

        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineTo(new Vector2d(20, 0))
                .addTemporalMarker(1, () -> {
                    // This marker runs one second into the trajectory
                    // Run your action in here!
                    targetPos = mid;
                })
                .build();

        // Define the angle to turn at
        double turnAngle1 = Math.toRadians(-270);

        // Third trajectory
        // We have to define a new end pose because we can't just call trajectory2.end()
        // Since there was a point turn before that
        // So we just take the pose from trajectory2.end(), add the previous turn angle to it
        Pose2d newLastPose = trajectory2.end().plus(new Pose2d(0, 0, turnAngle1));
        Trajectory trajectory3 = drive.trajectoryBuilder(newLastPose)
                .strafeTo(new Vector2d(10, 0))
                .build();

        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();



        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections) {
                    telemetry.addLine(String.format("\nDetected tag ID=%d", tag.id));
                    id = tag.id;
                }
            }
             else
                telemetry.addLine("Don't see tag :(");
            telemetry.update();
            sleep(20);


        }

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);


        while (opModeIsActive()) {

            //led
            pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            lights.setPattern(pattern);

            //apriltag
            if (id == 0)
                parkY = 32;
            else if (id == 1)
                parkY = 9;
            else if (id == 2)
                parkY = -17;

            // We essentially define the flow of the state machine through this switch statement
            switch (currentState) {
                case TRAJECTORY_1:
                    // Check if the drive class isn't busy
                    // `isBusy() == true` while it's following the trajectory
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished
                    // We move on to the next state
                    // Make sure we use the async follow function
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_2;
                        drive.followTrajectoryAsync(trajectory2);
                    }
                    break;
                case TRAJECTORY_2:
                    // Check if the drive class is busy following the trajectory
                    // Move on to the next state, TURN_1, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TURN_1;
                        drive.turnAsync(turnAngle1);
                    }
                    break;
                case TURN_1:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, TRAJECTORY_3, once finished
                    if (!drive.isBusy()) {
                        currentState = State.TRAJECTORY_3;
                        drive.followTrajectoryAsync(trajectory3);
                    }
                    break;
                case TRAJECTORY_3:
                    // Check if the drive class is busy following the trajectory
                    // If not, move onto the next state, WAIT_1
                    if (!drive.isBusy()) {
                        currentState = State.WAIT_1;

                        // Start the wait timer once we switch to the next state
                        // This is so we can track how long we've been in the WAIT_1 state
                        waitTimer1.reset();
                    }
                    break;
                case WAIT_1:
                    // Check if the timer has exceeded the specified wait time
                    // If so, move on to the TURN_2 state
                    if (waitTimer1.seconds() >= waitTime1) {
                        currentState = State.TURN_2;
                        drive.turnAsync(turnAngle1);
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }


           drive.update();
           lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = poseEstimate;

        }
    }
    // our lift class allows us to update the elevator without interrupting the drive code
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            elevator1 = hardwareMap.get(DcMotorEx.class, "elevator1");
            elevator2 = hardwareMap.get(DcMotorEx.class, "elevator2");

            elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void update() {

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
        }
    }

}