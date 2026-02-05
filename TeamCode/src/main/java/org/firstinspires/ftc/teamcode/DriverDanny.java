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
        //region shared poses
        public static Pose RED_GOAL_POSE = new Pose(144, 144, 0);
        public static Pose BLUE_GOAL_POSE = new Pose(0, 144, 0);
        public static Pose BLUE_FINAL_PARK_POSE = new Pose(105, 33, 0);
        public static Pose RED_FINAL_PARK_POSE = new Pose(38, 33, 0);
        //endregion

        //region BFA (Blue Far Auto)
        public static Pose BFA_START_POSE = new Pose(49, 8, Math.toRadians(90));
        public static Pose BFA_FAR_SHOOTING_POSE = new Pose(53, 14, Math.toRadians(111));
        public static Pose BFA_BOTTOM_ARTIFACTS_POSE = new Pose(45, 31, Math.toRadians(180));
        public static Pose BFA_EAT_BOTTOM_ARTIFACTS_POSE = new Pose(15, 31, Math.toRadians(180));
        public static Pose BFA_MIDDLE_ARTIFACTS_POSE = new Pose(45, 56, Math.toRadians(180));
        public static Pose BFA_EAT_MIDDLE_ARTIFACTS_POSE = new Pose(15, 56, Math.toRadians(180));
        public static Pose BFA_NEAR_SHOOTING_POSE = new Pose(49, 96, Math.toRadians(140));
        public static Pose BFA_TOP_ARTIFACTS_POSE = new Pose(45, 82, Math.toRadians(180));
        public static Pose BFA_EAT_TOP_ARTIFACTS_POSE = new Pose(22, 82, Math.toRadians(180));
        public static Pose BFA_FINAL_SHOOTING_POSE = new Pose(53, 107, Math.toRadians(144));
        //endregion

        //region RFA (Red Far Auto)
        public static Pose RFA_START_POSE = new Pose(95, 8, Math.toRadians(90));
        public static Pose RFA_FAR_SHOOTING_POSE = new Pose(91, 14, Math.toRadians(69));
        public static Pose RFA_BOTTOM_ARTIFACTS_POSE = new Pose(99, 31, Math.toRadians(0));
        public static Pose RFA_EAT_BOTTOM_ARTIFACTS_POSE = new Pose(129, 31, Math.toRadians(0));
        public static Pose RFA_MIDDLE_ARTIFACTS_POSE = new Pose(99, 56, Math.toRadians(0));
        public static Pose RFA_EAT_MIDDLE_ARTIFACTS_POSE = new Pose(129, 56, Math.toRadians(0));
        public static Pose RFA_NEAR_SHOOTING_POSE = new Pose(95, 96, Math.toRadians(45));
        public static Pose RFA_TOP_ARTIFACTS_POSE = new Pose(99, 82, Math.toRadians(0));
        public static Pose RFA_EAT_TOP_ARTIFACTS_POSE = new Pose(122, 82, Math.toRadians(0));
        public static Pose RFA_FINAL_SHOOTING_POSE = new Pose(91, 107, Math.toRadians(37));
        //endregion

        //region BBA (Blue Backup Auto)
        public static Pose BBA_START_POSE = new Pose(22, 123, Math.toRadians(144));
        public static Pose BBA_NEAR_SHOOTING_POSE = new Pose(49, 96, Math.toRadians(140));
        public static Pose BBA_TOP_ARTIFACTS_POSE = new Pose(47, 82, Math.toRadians(180));
        public static Pose BBA_EAT_TOP_ARTIFACTS_POSE = new Pose(22, 82, Math.toRadians(180));
        public static Pose BBA_MIDDLE_ARTIFACTS_POSE = new Pose(47, 59, Math.toRadians(180));
        public static Pose BBA_EAT_MIDDLE_ARTIFACTS_POSE = new Pose(15, 59, Math.toRadians(180));
        public static Pose BBA_BOTTOM_ARTIFACTS_POSE = new Pose(47, 37, Math.toRadians(180));
        public static Pose BBA_EAT_BOTTOM_ARTIFACTS_POSE = new Pose(15, 37, Math.toRadians(180));
        public static Pose BBA_FAR_SHOOTING_POSE = new Pose(53, 14, Math.toRadians(111));
        public static Pose BBA_END_POSE = new Pose(48, 55, Math.toRadians(180));
        //endregion

        //region RBA (Red Backup Auto)
        public static Pose RBA_START_POSE = new Pose(122, 123, Math.toRadians(37));
        public static Pose RBA_NEAR_SHOOTING_POSE = new Pose(95, 96, Math.toRadians(45));
        public static Pose RBA_TOP_ARTIFACTS_POSE = new Pose(99, 82, Math.toRadians(0));
        public static Pose RBA_EAT_TOP_ARTIFACTS_POSE = new Pose(122, 82, Math.toRadians(0));
        public static Pose RBA_MIDDLE_ARTIFACTS_POSE = new Pose(99, 56, Math.toRadians(0));
        public static Pose RBA_EAT_MIDDLE_ARTIFACTS_POSE = new Pose(129, 56, Math.toRadians(0));
        public static Pose RBA_BOTTOM_ARTIFACTS_POSE = new Pose(99, 31, Math.toRadians(0));
        public static Pose RBA_EAT_BOTTOM_ARTIFACTS_POSE = new Pose(129, 31, Math.toRadians(0));
        public static Pose RBA_FAR_SHOOTING_POSE = new Pose(91, 14, Math.toRadians(72));
        public static Pose RBA_END_POSE = new Pose(96, 55, Math.toRadians(0));
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
    //endregion

    //region Class Members
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private Limelight3A limelight;
    private Telemetry telemetry;
    private Follower follower; // part of the Pedro Pathing package, follows the path

    private boolean slowMode = false;
    private double limelightGoalHeadingError;
    private PIDFController headingPIDFController;
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

        lastKnownPose = this.getPose();

        telemetry.addData("CurrentXPos", lastKnownPose.getX());
        telemetry.addData("CurrentYPos", lastKnownPose.getY());
        telemetry.addData("CurrentHeading", Math.toDegrees(lastKnownPose.getHeading()));
        telemetry.addData("CurrentAlliance", currentAlliance.toString());
        telemetry.addData("CurrentDistanceFromGoal", this.getCurrentDistanceFromGoal());
        telemetry.addData("CurrentDriveMode", currentDriveMode.toString());
        telemetry.addData("SlowModeEnabled", slowMode);
    }

    public void updateLimeLight() {
        double currentHeading = this.follower.getHeading();
        limelight.updateRobotOrientation(Math.toDegrees(currentHeading)-90); // subtract 90 degrees here for pedropathing heading conversion

        limelightGoalHeadingError = -999;

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
                }
            }
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
            return Range.clip(headingPIDFController.run(), -0.3, 0.3);
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

        follower.followPath(newPath,0.89, holdEnd); // start the robot moving towards the new pose immediately
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
    //endregion
}