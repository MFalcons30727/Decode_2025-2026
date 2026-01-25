package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public class DriverDanny {
    public static class Poses {
        // put any poses we would ever need to go to in this list
        public static final Pose RED_FAR_START_POSE = new Pose(96, 8, Math.toRadians(90));
        public static final Pose BLUE_FAR_START_POSE = new Pose(49, 8, Math.toRadians(90));
        public static final Pose RED_BACKUP_START_POSE = new Pose(122, 125, Math.toRadians(40));
        public static final Pose BLUE_BACKUP_START_POSE = new Pose(22, 125, Math.toRadians(130));
        public static final Pose RED_TOP_ARTIFACTS_POSE = new Pose(100, 83, Math.toRadians(180));
        public static final Pose RED_MIDDLE_ARTIFACTS_POSE = new Pose(100, 60, Math.toRadians(180));

        public static final Pose RED_BOTTOM_ARTIFACTS_POSE = new Pose(100, 35, Math.toRadians(180));

        public static final Pose BLUE_TOP_ARTIFACTS_POSE = new Pose(45, 83, Math.toRadians(0));
        public static final Pose BLUE_MIDDLE_ARTIFACTS_POSE = new Pose(45, 60, Math.toRadians(0));
        public static final Pose BLUE_BOTTOM_ARTIFACTS_POSE = new Pose(45, 35, Math.toRadians(0));

        public static final Pose EAT_RED_TOP_ARTIFACTS_POSE = new Pose(125, 83, Math.toRadians(0));
        public static final Pose EAT_RED_MIDDLE_ARTIFACTS_POSE = new Pose(125, 60, Math.toRadians(0));
        public static final Pose EAT_RED_BOTTOM_ARTIFACTS_POSE = new Pose(125, 35, Math.toRadians(0));
        public static final Pose EAT_BLUE_TOP_ARTIFACTS_POSE = new Pose(18, 83, Math.toRadians(180));
        public static final Pose EAT_BLUE_MIDDLE_ARTIFACTS_POSE = new Pose(18, 60, Math.toRadians(180));
        public static final Pose EAT_BLUE_BOTTOM_ARTIFACTS_POSE = new Pose(18, 35, Math.toRadians(180));
        public static final Pose RED_NEAR_SHOOTING_POSE = new Pose (96, 96, Math.toRadians(40));
        public static final Pose BLUE_NEAR_SHOOTING_POSE = new Pose (49, 96, Math.toRadians(130));
        public static final Pose RED_FAR_SHOOTING_POSE = new Pose (85, 16, Math.toRadians(65));
        public static final Pose BLUE_FAR_SHOOTING_POSE = new Pose (60, 14, Math.toRadians(110));
        public static final Pose BLUE_FAR_END_POSE = new Pose (48, 132, Math.toRadians(90));
        public static final Pose RED_FAR_END_POSE = new Pose (95, 132, Math.toRadians(90));
        public static final Pose PARK_RED_GATE_POSE = new Pose (120,70, Math.toRadians(180));
        public static final Pose PARK_BLUE_GATE_POSE = new Pose (28,70, Math.toRadians(0));
        public static final Pose RED_GOAL_POSE = new Pose(144, 144, 0);
        public static final Pose BLUE_GOAL_POSE = new Pose(0, 144, 0);
        public static final Pose RED_FINAL_PARK_POSE = new Pose(38, 33, 0);
        public static final Pose BLUE_FINAL_PARK_POSE = new Pose(105, 33, 0);

    }

    public enum Alliance {
        RED,
        BLUE
    }

    public static Pose lastPose;

    private Follower follower; // part of the Pedro Pathing package, follows the path
    private Telemetry telemetry;
    private PathChain currentPath;// the current or most recent path we've built for the robot
    private double limelightGoalHeadingError;
    private Alliance currentAlliance;
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private Limelight3A limelight;
    private PIDFController headingController;
    private boolean shouldRelocalize;
    private double pedroX;
    private double pedroY;

    public DriverDanny(HardwareMap hardwareMap, Telemetry telemetryFromOpMode,
                       Alliance alliance, Pose startingPose) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // april tag pipeline
        limelight.setPollRateHz(100);
        limelight.start();

        // this is our constructor that gets called like this from our autos:  driver = new DriverDanny(hardwareMap, telemetry, DriverDanny.Poses.RED_FAR_START_POSE);
        // think of this like our "init" but for the DriverDanny specifically
        follower = Constants.createFollower(hardwareMap);

        //if (lastPose != null) {
            //follower.setStartingPose(lastPose);
        //} else {
            follower.setStartingPose(startingPose);
        //}

        telemetry = telemetryFromOpMode;
        currentAlliance = alliance;
        limelightGoalHeadingError = -999;

        // initialize a new PIDF controller using the heading coefficients we already tuned for auto
        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);

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
        // followed brogan's tutorial on this
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0; // this is speed, can be changed to lower if you want to let other drive during outreach events

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    public void fieldCentricDrive(double joyY, double joyX, double rotate) {
        double fieldX;
        double fieldY;
        double currentRobotHeading = this.getPose().getHeading();

        // change joystick "driver intent" to field X and Y intent
        if (currentAlliance == Alliance.BLUE) {
            fieldX = joyY; // if pressing down on joyY on blue alliance, intent is to increase field X
            fieldY = joyX; // if pressing right on joyX on blue alliance, intent is to increase field Y
        } else {
            fieldX = -joyY; // if pressing up on joyY on red alliance, intent is to increase field X
            fieldY = -joyX; // if pressing left on joyX on red alliance, intent is to increase field Y
        }

        // now translate field intent to robot actual direction
        double robotX = fieldX * Math.cos(currentRobotHeading)
                + fieldY * Math.sin(currentRobotHeading);

        double robotY = -fieldX * Math.sin(currentRobotHeading)
                + fieldY * Math.cos(currentRobotHeading);

        this.robotCentricDrive(robotX, robotY, rotate);
    }

    public double getHeadingErrorForAutoAimLimelight() {
        // Use deadband to protect against sign flipping near PI
        if (Math.abs(limelightGoalHeadingError) < 2) {
            headingController.updateError(0);
            return Range.clip(headingController.run(), -0.2, 0.2);
        } else if (limelightGoalHeadingError != -999) {
            headingController.updateError(limelightGoalHeadingError);
            return Range.clip(headingController.run(), -0.2, 0.2);
        } else {
            return -1;
        }
    }

    public double getHeadingErrorForAutoAimTrig() {
        Pose currentPose = this.getPose();
        Pose goalPose;

        if (currentAlliance == Alliance.RED) {
            goalPose = Poses.RED_GOAL_POSE;
        } else {
            goalPose = Poses.BLUE_GOAL_POSE;
        }

        // get the "vector" from current robot position to goal position
        double dx = goalPose.getX() - currentPose.getX();
        double dy = goalPose.getY() - currentPose.getY();

        // use atan2 to get heading from x-axis to goal in radians
        double targetHeading = Math.atan2(dy, dx);

        telemetry.addData("Target Heading", Math.toDegrees(targetHeading));

        double turnDirection = MathFunctions.getTurnDirection(currentPose.getHeading(), targetHeading);
        double angleDifference = MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), targetHeading);
        double headingError = turnDirection * angleDifference;

        telemetry.addData("Heading Error", Math.toDegrees(headingError));

        // Use deadband to protect against sign flipping near PI
        if (Math.abs(headingError) < Math.toRadians(1.5)) {
            headingController.updateError(0);
        } else {
            headingController.updateError(headingError);
        }

        // Use PIDF controller for smooth heading correction
        // Negate because robotCentricDrive treats +rotate as clockwise,
        // but PedroPathing's coordinate system uses +heading as counterclockwise
        return -Range.clip(headingController.run(), -0.5, 0.5);
    }

    public double getCurrentDistanceFromGoal() {
        if (currentAlliance == Alliance.RED) {
            return this.getPose().distanceFrom(Poses.RED_GOAL_POSE);
        } else {
            return this.getPose().distanceFrom(Poses.BLUE_GOAL_POSE);
        }
    }

    public Alliance getCurrentAlliance() {
        return currentAlliance;
    }

    public void swapCurrentAlliance() {
        if (currentAlliance == Alliance.BLUE) {
            currentAlliance = Alliance.RED;
        } else {
            currentAlliance = Alliance.BLUE;
        }
    }

    public void update() { // THIS MUST ALWAYS GO IN YOUR OPMODE LOOP EVERY CALL
        follower.update(); // this will just update the Pedro Pathing following but can add additional steps if we need to later
        this.updateLimeLight();

        lastPose = this.getPose();
        telemetry.addData("CurrentXPos", lastPose.getX());
        telemetry.addData("CurrentYPos", lastPose.getY());
        telemetry.addData("CurrentHeading", Math.toDegrees(lastPose.getHeading()));
        telemetry.addData("CurrentAlliance", currentAlliance.toString());
        telemetry.addData("CurrentDistanceFromGoal", this.getCurrentDistanceFromGoal());
        telemetry.addData("CurrentLLGoalHeadingError", limelightGoalHeadingError);
        telemetry.addData("LLPedroX", pedroX);
        telemetry.addData("LLPedroY", pedroY);
    }

    public Pose getPose() {
        return follower.getPose(); // this will just return the current pose from Pedro Pathing but can add additional steps if we need to later
    }

    public boolean isBusy() {
        return follower.isBusy(); // checks to see if PedroPathing is still busy and returns that result
    }

    public void moveToPose(Pose newPose, boolean holdEnd) {
        currentPath = follower.pathBuilder()
                .addPath(new BezierLine(getPose(), newPose))
                .setLinearHeadingInterpolation(getPose().getHeading(), newPose.getHeading(), 0.8)
                .build();

        follower.followPath(currentPath, holdEnd); // start the robot moving towards the new pose immediately
    }

    public void finalPark() {
        if (!this.isBusy()) {
            if (currentAlliance == Alliance.RED) {
                this.moveToPose(Poses.RED_FINAL_PARK_POSE, true);
            } else {
                this.moveToPose(Poses.BLUE_FINAL_PARK_POSE, true);
            }
        }
    }

    public void abortPath() {
        if (this.isBusy()) {
            follower.breakFollowing();
        }
    }

    public void relocalize() {
        shouldRelocalize = true;
    }

    public void updateLimeLight() {
        double currentHeading = this.follower.getHeading();

        // Learned that Tx, Ty, and Ta are degrees of error from tag, not meters.
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int tagID = fr.getFiducialId();

                if (currentAlliance == Alliance.BLUE && tagID == 20) {
                    limelightGoalHeadingError = fr.getTargetXDegrees();
                }
                else if (currentAlliance == Alliance.RED && tagID == 24) {
                    limelightGoalHeadingError = fr.getTargetXDegrees();
                } else {
                    limelightGoalHeadingError = -999;
                }
            }

            if (shouldRelocalize) {
                limelight.updateRobotOrientation(Math.toDegrees(currentHeading)-90); // subtract 90 degrees here for pedropathing heading conversion

                Pose3D botpose = result.getBotpose();

                if (botpose != null) {
                    // convert from meters to inches and adjust for 0,0 origin like pedropathing instead of -72,-72 that limelight uses
                    pedroX = (botpose.getPosition().y * 39.3700787) + 72; // x and y are intentionally flipped here
                    pedroY = (botpose.getPosition().x * 39.3700787) + 72;

                    Pose newPedroPose = new Pose(pedroX, pedroY, currentHeading);

                    this.follower.setPose(newPedroPose);
                    shouldRelocalize = false;
                }
            }
        }
    }
}
