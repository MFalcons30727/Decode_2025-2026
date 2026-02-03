package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public class DriverDanny {

    //region Poses
    public static class Poses {

        //region blue alliance poses
        //I checked all of these in the visualizer, they are all correct if the odometry is measured correctly.
        public static final Pose BLUE_FINAL_PARK_POSE = new Pose(105, 33, 0);
        public static final Pose BLUE_GOAL_POSE = new Pose(0, 144, 0);
        public static final Pose PARK_BLUE_GATE_POSE = new Pose (28,70, Math.toRadians(0));
        public static final Pose BLUE_FAR_SHOOTING_POSE = new Pose (53, 14, Math.toRadians(118));
        public static final Pose BLUE_FAR_END_POSE = new Pose (48, 132, Math.toRadians(90));
        public static final Pose BLUE_NEAR_SHOOTING_POSE = new Pose (49, 96, Math.toRadians(130));
        public static final Pose EAT_BLUE_TOP_ARTIFACTS_POSE = new Pose(17, 84, Math.toRadians(180));
        public static final Pose EAT_BLUE_MIDDLE_ARTIFACTS_POSE = new Pose(9, 60, Math.toRadians(180));
        public static final Pose EAT_BLUE_BOTTOM_ARTIFACTS_POSE = new Pose(9, 35, Math.toRadians(180));
        public static final Pose BLUE_TOP_ARTIFACTS_POSE = new Pose(45, 85, Math.toRadians(180));
        public static final Pose BLUE_MIDDLE_ARTIFACTS_POSE = new Pose(45, 60, Math.toRadians(180));
        public static final Pose BLUE_BOTTOM_ARTIFACTS_POSE = new Pose(45, 36, Math.toRadians(180));
        public static final Pose BLUE_BACKUP_START_POSE = new Pose(22, 125, Math.toRadians(130));
        public static final Pose BLUE_FAR_START_POSE = new Pose(49, 8, Math.toRadians(90));
        public static final Pose BLUE_TAP_OPEN_GATE_POSE = new Pose(19, 72, Math.toRadians(90));

        //endregions

        //region red alliance poses
        //I checked all of these in the visualizer, these poses are all correct as long as the odometry offsets are measured correctly.
        public static final Pose RED_FAR_START_POSE = new Pose(96, 8, Math.toRadians(90));
        public static final Pose RED_BACKUP_START_POSE = new Pose(122, 125, Math.toRadians(40));
        public static final Pose RED_TOP_ARTIFACTS_POSE = new Pose(100, 83, Math.toRadians(0));
        public static final Pose RED_MIDDLE_ARTIFACTS_POSE = new Pose(100, 60, Math.toRadians(0));
        public static final Pose RED_BOTTOM_ARTIFACTS_POSE = new Pose(100, 35, Math.toRadians(0));
        public static final Pose EAT_RED_TOP_ARTIFACTS_POSE = new Pose(128, 83, Math.toRadians(0));
        public static final Pose EAT_RED_MIDDLE_ARTIFACTS_POSE = new Pose(136, 58, Math.toRadians(0));
        public static final Pose EAT_RED_BOTTOM_ARTIFACTS_POSE = new Pose(136, 35, Math.toRadians(0));
        public static final Pose RED_NEAR_SHOOTING_POSE = new Pose (96, 96, Math.toRadians(40));
        public static final Pose RED_FAR_SHOOTING_POSE = new Pose (90, 16, Math.toRadians(65));
        public static final Pose RED_FAR_END_POSE = new Pose (95, 132, Math.toRadians(90));
        public static final Pose PARK_RED_GATE_POSE = new Pose (120,70, Math.toRadians(180));
        public static final Pose RED_GOAL_POSE = new Pose(144, 144, 0);
        public static final Pose RED_FINAL_PARK_POSE = new Pose(38, 33, 0);
        public static final Pose RED_TAP_OPEN_GATE_POSE = new Pose(126, 72, Math.toRadians(90));
        //endregion

    }
    //endregion

    //region Enums
    public enum Alliance {
        RED,
        BLUE
    }

    public enum DriveMode {
        FIELD,
        ROBOT
    }
    //endregion

    //region Static Variables
    public static Pose lastKnownPose;
    public static Alliance currentAlliance;
    public static DriveMode currentDriveMode;
    public static boolean inFarShootingZone = false;
    public static boolean inNearShootingZone = false;
    public static boolean isAlignedToGoal = false;
    public static ElapsedTime idleTimer;
    //endregion

    //region Class Members
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private Limelight3A limelight;
    private Telemetry telemetry;
    private Follower follower; // part of the Pedro Pathing package, follows the path

    private boolean slowMode = false;
    private double limelightGoalHeadingError;
    private PIDFController headingPIDFController;
    //private boolean shouldRelocalize = false;
    private double relocalizePedroX;
    private double relocalizePedroY;
    //endregion

    //region Constructors
    public DriverDanny(HardwareMap hardwareMap, Telemetry telemetryFromOpMode,
                       Alliance alliance, Pose startingPose) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // april tag pipeline
        limelight.setPollRateHz(100);
        limelight.start();

        // this is our constructor that gets called like this from our autos:  driver = new DriverDanny(hardwareMap, telemetry, DriverDanny.Poses.RED_FAR_START_POSE);
        // think of this like our "init" but for the DriverDanny specifically
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);

        telemetry = telemetryFromOpMode;
        currentAlliance = alliance;
        currentDriveMode = DriveMode.FIELD;
        limelightGoalHeadingError = -999;
        inFarShootingZone = false;
        inNearShootingZone = false;
        isAlignedToGoal = false;

        idleTimer = new ElapsedTime();

        // initialize a new PIDF controller using the heading coefficients we already tuned for auto
        headingPIDFController = new PIDFController(follower.constants.coefficientsHeadingPIDF);

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
    //endregion

    //region Update Functions
    public void update() { // THIS MUST ALWAYS GO IN YOUR OPMODE LOOP EVERY CALL
        follower.update(); // this will just update the Pedro Pathing following but can add additional steps if we need to later
        this.updateLimeLight(); // should update our limelight every loop

        // if the robot has changed position, reset the timer so we can track how long we've been idle
        if (lastKnownPose != null && lastKnownPose.distanceFrom(this.getPose()) > 0.5) {
            idleTimer.reset();
        }

        lastKnownPose = this.getPose();

        checkForFarShootZone(lastKnownPose.getX(), lastKnownPose.getY(), 9);
        checkForNearShootZone(lastKnownPose.getX(), lastKnownPose.getY(), 9);

        if (Math.abs(limelightGoalHeadingError) < 2) {
            isAlignedToGoal = true;
        } else {
            isAlignedToGoal = false;
        }

        telemetry.addData("CurrentXPos", lastKnownPose.getX());
        telemetry.addData("CurrentYPos", lastKnownPose.getY());
        telemetry.addData("CurrentHeading", Math.toDegrees(lastKnownPose.getHeading()));
        telemetry.addData("CurrentAlliance", currentAlliance.toString());
        telemetry.addData("CurrentDistanceFromGoal", this.getCurrentDistanceFromGoal());
        telemetry.addData("CurrentDriveMode", currentDriveMode.toString());
        telemetry.addData("SlowModeEnabled", slowMode);
        //telemetry.addData("LLPedroX", relocalizePedroX);
        //telemetry.addData("LLPedroY", relocalizePedroY);
    }

    public void updateLimeLight() {
        double currentHeading = this.follower.getHeading();
        limelight.updateRobotOrientation(Math.toDegrees(currentHeading)-90); // subtract 90 degrees here for pedropathing heading conversion

        // Learned that Tx, Ty, and Ta are degrees of error from tag, not meters.
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                int tagID = fr.getFiducialId();

                if (currentAlliance == Alliance.BLUE && tagID == 20) {
                    limelightGoalHeadingError = fr.getTargetXDegrees();
                    break;
                }
                else if (currentAlliance == Alliance.RED && tagID == 24) {
                    limelightGoalHeadingError = fr.getTargetXDegrees();
                    break;
                } else {
                    limelightGoalHeadingError = -999;
                }
            }

//            if (shouldRelocalize) {
//                Pose3D botpose = result.getBotpose_MT2();
//
//                if (botpose != null) {
//                    // convert from meters to inches and adjust for 0,0 origin like pedropathing instead of -72,-72 that limelight uses
//                    relocalizePedroX = (botpose.getPosition().y * 39.3700787) + 72; // x and y are intentionally flipped here
//                    relocalizePedroY = (botpose.getPosition().x * 39.3700787) + 72;
//
//                    Pose newPedroPose = new Pose(relocalizePedroX, relocalizePedroY, currentHeading);
//
//                    this.follower.setPose(newPedroPose);
//                    shouldRelocalize = false;
//                }
//            }
        }
    }
    //endregion

    //region Drive Functions
    public void drive(double joyY, double joyX, double rotate) {
        if (currentDriveMode == DriveMode.FIELD) {
            fieldCentricDrive(joyY, joyX, rotate);
        } else {
            robotCentricDrive(joyY, joyX, rotate);
        }
    }
    private void robotCentricDrive(double forward, double strafe, double rotate) {
        // followed brogan's tutorial on this
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        // this is useful when precision driving is needed (like parking adjustments)
        if (slowMode) { maxSpeed = 0.5; }

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }

    private void fieldCentricDrive(double joyY, double joyX, double rotate) {
        double fieldX;
        double fieldY;
        double currentRobotHeading = this.follower.getHeading();

        // change joystick "driver intent" to field X and Y intent
        if (currentAlliance == Alliance.BLUE) {
            fieldX = -joyY; // if pressing down on joyY on blue alliance, intent is to increase field X
            fieldY = joyX; // if pressing right on joyX on blue alliance, intent is to increase field Y
        } else {
            fieldX = joyY; // if pressing up on joyY on red alliance, intent is to increase field X
            fieldY = -joyX; // if pressing left on joyX on red alliance, intent is to increase field Y
        }

        // now translate field intent to robot actual direction
        double robotX = fieldX * Math.cos(currentRobotHeading)
                + fieldY * Math.sin(currentRobotHeading);

        double robotY = fieldX * Math.sin(currentRobotHeading)
                - fieldY * Math.cos(currentRobotHeading);

        // If our currentDriveMode is robotCentric, ignore the translations and use original paramters
        if (currentDriveMode == DriveMode.ROBOT) {
            this.robotCentricDrive(joyY, joyX, rotate);
        } else {
            this.robotCentricDrive(robotX, robotY, rotate);
        }
    }
    //endregion

    //region Auto-aim Functions
    public double getHeadingErrorForAutoAimLimelight() {
        // Use deadband to protect against sign flipping near PI
        if (Math.abs(limelightGoalHeadingError) < 2) {
            headingPIDFController.updateError(0);
            return Range.clip(headingPIDFController.run(), -0.2, 0.2);
        } else if (limelightGoalHeadingError != -999) {
            headingPIDFController.updateError(limelightGoalHeadingError);
            return Range.clip(headingPIDFController.run(), -0.2, 0.2);
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
        //telemetry.addData("Target Heading", Math.toDegrees(targetHeading));

        double turnDirection = MathFunctions.getTurnDirection(currentPose.getHeading(), targetHeading);
        double angleDifference = MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), targetHeading);
        double headingError = turnDirection * angleDifference;
        //telemetry.addData("Heading Error", Math.toDegrees(headingError));

        // Use deadband to protect against sign flipping near PI
        if (Math.abs(headingError) < Math.toRadians(1.5)) {
            headingPIDFController.updateError(0);
        } else {
            headingPIDFController.updateError(headingError);
        }

        // Use PIDF controller for smooth heading correction
        // Negate because robotCentricDrive treats +rotate as clockwise,
        // but PedroPathing's coordinate system uses +heading as counterclockwise
        return -Range.clip(headingPIDFController.run(), -0.5, 0.5);
    }
    //endregion

    //region Other Helper Functions
    public double getCurrentDistanceFromGoal() {
        if (currentAlliance == Alliance.RED) {
            return this.getPose().distanceFrom(Poses.RED_GOAL_POSE);
        } else {
            return this.getPose().distanceFrom(Poses.BLUE_GOAL_POSE);
        }
    }

    public void swapCurrentAlliance() {
        if (currentAlliance == Alliance.BLUE) {
            currentAlliance = Alliance.RED;
        } else {
            currentAlliance = Alliance.BLUE;
        }
    }

    public void swapCurrentDriveMode() {
        if (currentDriveMode == DriveMode.FIELD) {
            currentDriveMode = DriveMode.ROBOT;
        } else {
            currentDriveMode = DriveMode.FIELD;
        }
    }

    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    public Pose getPose() {
        return follower.getPose(); // this will just return the current pose from Pedro Pathing but can add additional steps if we need to later
    }

    public boolean isBusy() {
        return follower.isBusy(); // checks to see if PedroPathing is still busy and returns that result
    }

    public void moveToPose(Pose newPose, boolean holdEnd) {
        PathChain newPath = follower.pathBuilder()
                .addPath(new BezierLine(getPose(), newPose))
                .setLinearHeadingInterpolation(getPose().getHeading(), newPose.getHeading(), 0.8)
                .build();

        follower.followPath(newPath,0.67, holdEnd); // start the robot moving towards the new pose immediately
    }

    public void finalPark() {
        if (!this.isBusy()) {
            if (currentAlliance == Alliance.RED) {
                this.moveToPose(Poses.RED_FINAL_PARK_POSE, false);
            } else {
                this.moveToPose(Poses.BLUE_FINAL_PARK_POSE, false);
            }
        }
    }

    public void abortPath() {
        if (this.isBusy()) {
            follower.breakFollowing();
        }
    }

    // to localize right now, make sure to drive into your alliance corner and make sure the robot is facing upfield towards the goals (90 degrees)
    public void relocalize() {
        //shouldRelocalize = true;
        if (currentAlliance == Alliance.RED) {
            this.follower.setPose(new Pose(8.5, 8.5, Math.toRadians(90)));
        } else {
            this.follower.setPose(new Pose(135.5, 8.5, Math.toRadians(90)));
        }
    }

    public void checkForNearShootZone(double x, double y, double buffer) {
        if ((y <= 144 + buffer) && (y >= -x + 144 - buffer) && (y >= x - buffer)) {
            inNearShootingZone = true;
        } else {
            inNearShootingZone = false;
        }
    }

    public void checkForFarShootZone(double x, double y, double buffer) {
        if ((y >= 0 - buffer) && (y <= x - 48 + buffer) && (y <= -x + 96 + buffer)) {
            inFarShootingZone = true;
        } else {
            inFarShootingZone = false;
        }
    }
    //endregion
}
