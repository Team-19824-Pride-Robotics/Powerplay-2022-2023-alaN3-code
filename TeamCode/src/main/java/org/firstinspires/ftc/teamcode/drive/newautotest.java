package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


import org.firstinspires.ftc.teamcode.drive.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;

/**
 * This opmode explains how you follow multiple trajectories in succession, asynchronously. This
 * allows you to run your own logic beside the drive.update() command. This enables one to run
 * their own loops in the background such as a PID controller for a lift. We can also continuously
 * write our pose to PoseStorage.
 * <p>
 * The use of a State enum and a currentState field constitutes a "finite state machine."
 * You should understand the basics of what a state machine is prior to reading this opmode. A good
 * explanation can be found here:
 * https://www.youtube.com/watch?v=Pu7PMN5NGkQ (A finite state machine introduction tailored to FTC)
 * or here:
 * https://gm0.org/en/stable/docs/software/finite-state-machines.html (gm0's article on FSM's)
 * <p>
 * You can expand upon the FSM concept and take advantage of command based programming, subsystems,
 * state charts (for cyclical and strongly enforced states), etc. There is still a lot to do
 * to supercharge your code. This can be much cleaner by abstracting many of these things. This
 * opmode only serves as an initial starting point.
 */

@Config
@Autonomous(group = "advanced")
public class newautotest extends LinearOpMode {

    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        CLAWOPEN,
        CLAWCLOSE,
        ARMRIGHT,
        ARMYMIDDLE,
        TRAJECTORY_1,   // First, follow a splineTo() trajectory
        TRAJECTORY_2,   // Then, follow a lineTo() trajectory
        TURN_1,         // Then we want to do a point turn
        TRAJECTORY_3,   // Then, we follow another lineTo() trajectory
        TRAJECTORY_4,
        CONSTACK,
        JUNCTION,
        WAIT_1,         // Then we're gonna wait a second
        TURN_2,         // Finally, we're gonna turn again
        IDLE            // Our bot will enter the IDLE state when done
    }

    // We define the current state we're on
    // Default to IDLE
    State currentState = State.IDLE;

    // Define our start pose
    // This assumes we start at x: 15, y: 10, heading: 180 degrees
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

    //junction
    public static int top = -2000;
    public static int mid = -1350;
    public static int low = -850;
    public static int pickup = -20;

    public static double turn = 90;
    // to first pole
    public static double x1 = 43;
    public static double y1 = -.5;
    //move up to line up for pickup
    public static double x2 = 50;
    public static double y2 = -.5;
    //cone stack location
    public static double x3 = 52;
    public static double y3 = 25.5;
    //backup to score
    public static double x4 = 48.8;
    public static double y4 = -8.48;
    //score last cone on high
    public static double x5 = 48.5;
    public static double y5 = -8.6;
    // score second cone on high
    public static double x7 = 48.7;
    public static double y7 = -8.7;
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

    //pid
    public static double Kp = .04;
    public static double Ki = 0;
    public static double Kd = .00015;

    public static double targetPos = 0;
    double integralSum = 0;
    double lastError = 0;
    double error = 0;
    double encoderPosition = 0;
    double derivative = 0;
    public static double power = 0;

    DcMotorEx elevator1;
    DcMotorEx elevator2;
    Servo servo1;
    Servo servo3;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize our lift
        Lift lift = new Lift(hardwareMap);

        // Initialize SampleMecanumDrive
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set inital pose
        drive.setPoseEstimate(startPose);

        // Let's define our trajectories
        Trajectory CLAWCLOSE = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    servo1.setPosition(sr1c);
                })
                .build();
        Trajectory CLAWOPEN = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    servo1.setPosition(sr1o);
                })
                .build();
        Trajectory ARMRIGHT = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    servo3.setPosition(ar);
                })
                .build();
        Trajectory ARMMIDDLE = drive.trajectoryBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    servo3.setPosition(armMiddle);
                })
                .build();

        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(x1, y1, Math.toRadians(0)))
                .build();
        // Second trajectory
        // Ensure that we call trajectory1.end() as the start for this one
        Trajectory trajectory2 = drive.trajectoryBuilder(trajectory1.end())
                .lineToLinearHeading(new Pose2d(x6, y6, Math.toRadians(0)))
                .build();

        Trajectory trajectory3 = drive.trajectoryBuilder(trajectory2.end())
                .lineToLinearHeading(new Pose2d(x2,y2, Math.toRadians(0)))
                .build();

        double turnAngle1 = Math.toRadians(turn);

        Pose2d newLastPose = trajectory2.end().plus(new Pose2d(0, 0, turnAngle1));

        Trajectory trajectory4 = drive.trajectoryBuilder(newLastPose)
                .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(turn)))
                .build();

        Trajectory JUNCTION = drive.trajectoryBuilder(trajectory4.end())
                .lineToLinearHeading(new Pose2d(x4, y4, Math.toRadians(turn)))
                .build();

        Trajectory CONESTACK = drive.trajectoryBuilder(JUNCTION.end())
                .lineToLinearHeading(new Pose2d(x3, y3, Math.toRadians(turn)))
                .build();




        // Define a 1.5 second wait time
        double waitTime1 = 1.5;
        ElapsedTime waitTimer1 = new ElapsedTime();

        // Define the angle for turn 2
        double turnAngle2 = Math.toRadians(720);

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to TRAJECTORY_1, our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentState = State.TRAJECTORY_1;
        drive.followTrajectoryAsync(trajectory1);

        while (opModeIsActive() && !isStopRequested()) {
            // Our state machine logic
            // You can have multiple switch statements running together for multiple state machines
            // in parallel. This is the basic idea for subsystems and commands.

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
                        drive.turnAsync(turnAngle2);
                    }
                    break;
                case TURN_2:
                    // Check if the drive class is busy turning
                    // If not, move onto the next state, IDLE
                    // We are done with the program
                    if (!drive.isBusy()) {
                        currentState = State.IDLE;
                    }
                    break;
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }

            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            drive.update();
            // We update our lift PID continuously in the background, regardless of state
            lift.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Continually write pose to `PoseStorage`
          //  PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    // Assume we have a hardware class called lift
    // Lift uses a PID controller to maintain its height
    // Thus, update() must be called in a loop
    class Lift {
        public Lift(HardwareMap hardwareMap) {
            // Beep boop this is the the constructor for the lift
            // Assume this sets up the lift hardware

            elevator1 = hardwareMap.get(DcMotorEx.class, "elevator1");
            elevator2 = hardwareMap.get(DcMotorEx.class, "elevator2");

            elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void update() {
            // Beep boop this is the lift update function
            // Assume this runs some PID controller for the lift
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