package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriverDanny {
    public static class Poses {
        // put any poses we would ever need to go to in this list
        public static final Pose RED_FAR_START_POSE = new Pose(96, 8, Math.toRadians(90));
        public static final Pose BLUE_FAR_START_POSE = new Pose(49, 8, Math.toRadians(90));
        public static final Pose RED_BACKUP_START_POSE = new Pose(122, 125, Math.toRadians(40));
        public static final Pose BLUE_BACKUP_START_POSE = new Pose(22, 125, Math.toRadians(130));
        public static final Pose RED_NEAR_SHOOTING_POSE = new Pose (96, 96, Math.toRadians(40));
        public static final Pose BLUE_NEAR_SHOOTING_POSE = new Pose (49, 96, Math.toRadians(130));
        public static final Pose RED_FAR_SHOOTING_POSE = new Pose (80, 16, Math.toRadians(65));
        public static final Pose BLUE_FAR_SHOOTING_POSE = new Pose (62, 14, Math.toRadians(110));
        public static final Pose PARK_RED_GATE_POSE = new Pose (120,70, Math.toRadians(180));
        public static final Pose PARK_BLUE_GATE_POSE = new Pose (28,70, Math.toRadians(0));
        public static final Pose RED_GOAL_POSE = new Pose(144, 144, Math.toRadians(225));
        public static final Pose BLUE_GOAL_POSE = new Pose(0, 144, Math.toRadians(315));
    }

    public enum Team {
        RED,
        BLUE
    }
    private Follower follower; // part of the Pedro Pathing package, follows the path
    private Telemetry telemetry;
    private PathChain currentPath; // the current or most recent path we've built for the robot
    private Team currentTeam;
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;



    public DriverDanny(HardwareMap hardwareMap, Telemetry telemetryFromOpMode, Pose startingPose, Team team) {
        // this is our constructor that gets called like this from our autos:  driver = new DriverDanny(hardwareMap, telemetry, DriverDanny.Poses.RED_FAR_START_POSE);
        // think of this like our "init" but for the DriverDanny specifically
        follower = Constants.createFollower(hardwareMap);  // TODO: need to retune Pedro Pathing constants with the new bot
        telemetry = telemetryFromOpMode;
        follower.setStartingPose(startingPose);
        currentTeam = team;

        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftRear");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightRear");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public void robotCentricDrive(double forward, double strafe, double rotate) {

        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0; //this is speed, can be changed to lower if you want to let other drive during outreach events

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));


    }

    public void fieldCentricDrive(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - follower.getHeading());

        double newFoward = r * Math.sin(theta);
        double newStrafe = Math.cos(theta);

        this.robotCentricDrive(newFoward, newStrafe, rotate);

    }

    public void update() { // THIS MUST ALWAYS GO IN YOUR OPMODE LOOP EVERY CALL
        follower.update(); // this will just update the Pedro Pathing following but can add additional steps if we need to later

        Pose currentPose = getPose();
        telemetry.addData("CurrentXPos", currentPose.getX());
        telemetry.addData("CurrentYPos", currentPose.getY());
        telemetry.addData("CurrentHeading", currentPose.getHeading());
        telemetry.addData("CurrentDistanceFromGoal", getCurrentDistanceFromGoal());
    }

    public Pose getPose() {
        return follower.getPose(); // this will just return the current pose from Pedro Pathing but can add additional steps if we need to later
    }

    /**
     * See if the robot is still in the process of moving to a new pose
     *
     * @return whether or not the robot is still moving to the new pose
     */
    public boolean isBusy() {
        return follower.isBusy(); // checks to see if PedroPathing is still busy and returns that result
    }

    /**
     * Moves the robot from our current position to the new pose we pass as a parameter here
     *
     * @param newPose the new pose to move to
     * @param holdEnd whether or not to have PedroPathing hold the robot at the newPose when done
     */
    public void moveToPose(Pose newPose, boolean holdEnd) {
        currentPath = follower.pathBuilder()
                .addPath(new BezierLine(getPose(), newPose))
                .setLinearHeadingInterpolation(getPose().getHeading(), newPose.getHeading())
                .build();

        follower.followPath(currentPath, holdEnd); // start the robot moving towards the new pose immediately
    }

    public double getCurrentDistanceFromGoal() {
        if (currentTeam == Team.RED) {
            return getPose().distanceFrom(Poses.RED_GOAL_POSE);
        } else {
            return getPose().distanceFrom(Poses.BLUE_GOAL_POSE);
        }
    }
}
