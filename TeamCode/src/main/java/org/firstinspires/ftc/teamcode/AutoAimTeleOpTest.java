package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// made for blue goal auto aim.  will need to think about red goal
@TeleOp (name="AutoAimTeleOpTest", group = "TeleOp")
public class AutoAimTeleOpTest extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Timer pathTimer;
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private final Pose blueFarStartPose = new Pose(49, 8, Math.toRadians(90)); // put bot on blue far start pose. not sure if this is needed?
    private final Pose blueGoalPose = new Pose(0, 144, Math.toRadians(90)); // the back of the blue goal that we're aiming at
    private Follower follower; // part of the Pedro Pathing package, follows the path
    private ShooterMcGavin shooter;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightRear");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(blueFarStartPose);

        shooter = new ShooterMcGavin(hardwareMap, telemetry);

        // wait for start (then player presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; //remember that Y stick value is reversed!!
        double x = gamepad1.right_stick_x * 1.1; //counteract thingy
        double rx = gamepad1.left_stick_x;

        //denominator is max power, for right now we changed it from 1 to 0.5, or absolute value

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backRightPower);
        backRightDrive.setPower(backLeftPower);

        if(gamepad2.left_bumper) {
            // if left bumper held down, make robot face blue goal
            Pose currentPose = follower.getPose();

            // get difference in X/Y coordinates from the goal
            double dx = blueGoalPose.getX() - currentPose.getX();
            double dy = blueGoalPose.getY() - currentPose.getY();

            // use atan2 to get heading adjustment in radians and then convert to degrees
            double targetHeading = Math.toDegrees(Math.atan2(dy, dx));

            // get current heading
            double currentHeading = Math.toDegrees(currentPose.getHeading());

            // get how much we need to change our heading
            double degreesError = targetHeading - currentHeading;

            // need to know whether it's shorter to turn left or right.  tutorial called this "normalization"
            degreesError = ((degreesError + 180) % 360) - 180;

            // if angleError > 0, turn left; else turn right
            boolean isLeft = degreesError > 0;

            // use pedro pathing to turn towards target
            follower.turnDegrees(targetHeading, isLeft);
        }

        if(gamepad2.right_trigger > 0) {
            // pedro pathing apparently has this cool function that returns the "straight-line" distance between two poses.
            // i think this is inches but not sure
            // going to try and use it to set our velocity later
            // if we're not within one of our measured shooting distances where we know the velocity, don't shoot
//            double distanceFromGoal = follower.getPose().distanceFrom(blueGoalPose);
//            if (distanceFromGoal >= 45 && distanceFromGoal < 65) {
//                shooter.startShooting(1200);
//            } else if (distanceFromGoal >= 65 && distanceFromGoal < 75) {
//                shooter.startShooting(1250);
//            } else if (distanceFromGoal >= 75 && distanceFromGoal < 85) {
//                shooter.startShooting(1300);
//            }

            shooter.startShooting();
        }

        follower.update();
        shooter.update();

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.update();
    }
}

