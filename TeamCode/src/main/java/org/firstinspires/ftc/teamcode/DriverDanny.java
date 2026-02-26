package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.util.PoseHistory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

public class DriverDanny {

    //region Poses
    public static class Poses {
        //region shared poses
        public static Pose RED_GOAL_POSE = new Pose(144, 144, Math.toRadians(0));
        public static Pose RED_GOAL_AIMING_POSE = new Pose(134, 134, Math.toRadians(0));
        public static Pose BLUE_GOAL_POSE = new Pose(0, 144, Math.toRadians(0));
        public static Pose BLUE_GOAL_AIMING_POSE = new Pose(6, 134, Math.toRadians(0));
        public static Pose BLUE_FINAL_PARK_POSE = new Pose(105, 33, Math.toRadians(0));
        public static Pose RED_FINAL_PARK_POSE = new Pose(38, 33, Math.toRadians(0));
        public static Pose BLUE_OPEN_GATE_POSE = new Pose (23, 75,Math.toRadians(90));
        public static Pose RED_OPEN_GATE_POSE = new Pose (120, 75,Math.toRadians(90));
        //endregion

        //region BFA (Blue Far Auto)
        public static Pose TEST_POSE1 = new Pose(72-8, 72+12, Math.toRadians(90));
        public static Pose TEST_POSE2 = new Pose(72+8, 72-12, Math.toRadians(90));
        public static Pose TEST_POSE3 = new Pose(72-8, 72+12, Math.toRadians(144));
        public static Pose TEST_POSE4 = new Pose(72+8, 72-12, Math.toRadians(144));


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
        public static Pose BBA_AVOID_RUNNING_INTO_GATE_TOP_ARTIFACTS_POSE = new Pose(44, 82, Math.toRadians(180));
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
        public static Pose RBA_TOP_ARTIFACTS_POSE = new Pose(97, 84, Math.toRadians(0));
        public static Pose RBA_AVOID_RUNNING_INTO_GATE_TOP_ARTIFACTS_POSE = new Pose(100, 84, Math.toRadians(0));
        public static Pose RBA_EAT_TOP_ARTIFACTS_POSE = new Pose(120, 84, Math.toRadians(0));
        public static Pose RBA_MIDDLE_ARTIFACTS_POSE = new Pose(97, 59, Math.toRadians(0));
        public static Pose RBA_EAT_MIDDLE_ARTIFACTS_POSE = new Pose(129, 59, Math.toRadians(0));
        public static Pose RBA_BOTTOM_ARTIFACTS_POSE = new Pose(97, 42, Math.toRadians(0));
        public static Pose RBA_EAT_BOTTOM_ARTIFACTS_POSE = new Pose(129, 42, Math.toRadians(0));
        public static Pose RBA_FAR_SHOOTING_POSE = new Pose(91, 14, Math.toRadians(72));
        public static Pose RBA_END_POSE = new Pose(96, 55, Math.toRadians(0));
        //endregion

        //region ANBBA (All Near Blue Backup Auto)
        public static Pose ANBBA_START_POSE = new Pose(22, 123, Math.toRadians(144));
        public static Pose ANBBA_NEAR_SHOOTING_POSE = new Pose(49, 96, Math.toRadians(140));
        public static Pose ANBBA_TOP_ARTIFACTS_POSE = new Pose(47, 82, Math.toRadians(180));
        public static Pose ANBBA_EAT_TOP_ARTIFACTS_POSE = new Pose(22, 82, Math.toRadians(180));
        public static Pose ANBBA_MIDDLE_ARTIFACTS_POSE = new Pose(47, 59, Math.toRadians(180));
        public static Pose ANBBA_EAT_MIDDLE_ARTIFACTS_POSE = new Pose(15, 59, Math.toRadians(180));
        public static Pose ANBBA_BOTTOM_ARTIFACTS_POSE = new Pose(47, 39, Math.toRadians(180));
        public static Pose ANBBA_EAT_BOTTOM_ARTIFACTS_POSE = new Pose(15, 39, Math.toRadians(180));
        public static Pose ANBBA_FINAL_SHOOTING_POSE = new Pose(53, 113, Math.toRadians(144));
        //endregion

        //region ANRBA (All Near Red Backup Auto)
        public static Pose ANRBA_START_POSE = new Pose(122, 123, Math.toRadians(37));
        public static Pose ANRBA_NEAR_SHOOTING_POSE = new Pose(95, 96, Math.toRadians(45));
        public static Pose ANRBA_TOP_ARTIFACTS_POSE = new Pose(99, 82, Math.toRadians(0));
        public static Pose ANRBA_EAT_TOP_ARTIFACTS_POSE = new Pose(122, 82, Math.toRadians(0));
        public static Pose ANRBA_MIDDLE_ARTIFACTS_POSE = new Pose(99, 56, Math.toRadians(0));
        public static Pose ANRBA_EAT_MIDDLE_ARTIFACTS_POSE = new Pose(129, 56, Math.toRadians(0));
        public static Pose ANRBA_BOTTOM_ARTIFACTS_POSE = new Pose(99, 31, Math.toRadians(0));
        public static Pose ANRBA_EAT_BOTTOM_ARTIFACTS_POSE = new Pose(129, 31, Math.toRadians(0));
        public static Pose ANRBA_FINAL_SHOOTING_POSE = new Pose(91, 107, Math.toRadians(37));
        //endregion

        //region AFBFA (All Far Blue Far Auto)
        public static Pose AFBFA_START_POSE = new Pose(49, 8, Math.toRadians(90));
        public static Pose AFBFA_FAR_SHOOTING_POSE = new Pose(53, 14, Math.toRadians(111));
        public static Pose AFBFA_BOTTOM_ARTIFACTS_POSE = new Pose(45, 31, Math.toRadians(180));
        public static Pose AFBFA_EAT_BOTTOM_ARTIFACTS_POSE = new Pose(15, 31, Math.toRadians(180));
        public static Pose AFBFA_MIDDLE_ARTIFACTS_POSE = new Pose(45, 56, Math.toRadians(180));
        public static Pose AFBFA_EAT_MIDDLE_ARTIFACTS_POSE = new Pose(15, 56, Math.toRadians(180));
        public static Pose AFBFA_TOP_ARTIFACTS_POSE = new Pose(45, 82, Math.toRadians(180));
        public static Pose AFBFA_EAT_TOP_ARTIFACTS_POSE = new Pose(22, 82, Math.toRadians(180));
        public static Pose AFBFA_END_POSE = new Pose(48, 50, Math.toRadians(180));
        public static Pose AFBFA_HUMAN_PLAYER_AREA_ARTIFACTS_POSE = new Pose(16, 20, Math.toRadians(120));
        public static Pose AFBFA_EAT_HUMAN_PLAYER_AREA_ARTIFACTS_POSE = new Pose(12, 33, Math.toRadians(120));
        public static Pose AFBFA_HUMAN_PLAYER_AREA_ARTIFACTS_SECOND_TIME_POSE = new Pose(12, 8, Math.toRadians(180));
        public static Pose AFBFA_EAT_HUMAN_PLAYER_AREA_ARTIFACTS_SECOND_TIME_POSE = new Pose(8, 8, Math.toRadians(180));
        //endregion

        //region AFRFA (All Far Red Far Auto)
        public static Pose AFRFA_START_POSE = new Pose(95, 8, Math.toRadians(90));
        public static Pose AFRFA_FAR_SHOOTING_POSE = new Pose(91, 14, Math.toRadians(69));
        public static Pose AFRFA_BOTTOM_ARTIFACTS_POSE = new Pose(99, 31, Math.toRadians(0));
        public static Pose AFRFA_EAT_BOTTOM_ARTIFACTS_POSE = new Pose(131, 31, Math.toRadians(0));
        public static Pose AFRFA_MIDDLE_ARTIFACTS_POSE = new Pose(99, 56, Math.toRadians(0));
        public static Pose AFRFA_EAT_MIDDLE_ARTIFACTS_POSE = new Pose(131, 56, Math.toRadians(0));
        public static Pose AFRFA_TOP_ARTIFACTS_POSE = new Pose(99, 82, Math.toRadians(0));
        public static Pose AFRFA_EAT_TOP_ARTIFACTS_POSE = new Pose(122, 82, Math.toRadians(0));
        public static Pose AFRFA_END_POSE = new Pose(96, 55, Math.toRadians(0));
        public static Pose AFRFA_HUMAN_PLAYER_AREA_ARTIFACTS_POSE = new Pose(128, 20, Math.toRadians(75));
        public static Pose AFRFA_EAT_HUMAN_PLAYER_AREA_ARTIFACTS_POSE = new Pose(132, 33, Math.toRadians(75));
        public static Pose AFRFA_HUMAN_PLAYER_AREA_ARTIFACTS_SECOND_TIME_POSE = new Pose(125, 8, Math.toRadians(0));
        public static Pose AFRFA_EAT_HUMAN_PLAYER_AREA_ARTIFACTS_SECOND_TIME_POSE = new Pose(130, 8, Math.toRadians(0));

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
    public static ElapsedTime lastRelocalizeTimer;

    private static final Style robotLook = new Style("", "#3F51B5", 0.75);
    private static final Style historyLook = new Style("", "#4CAF50", 0.75);
    private static final Style limelightLook = new Style("", "#FF9800", 0.75);
    //endregion

    //region Class Members
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private Limelight3A limelight;
    private Telemetry telemetry;
    private Follower follower; // part of the Pedro Pathing package, follows the path

    public boolean debug = false;
    private boolean slowMode = false;
    private PIDFController headingPIDFController;

    private boolean shouldRelocalize = false;
    private double lastAutoAimHeadingError = 999;
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

        PanelsField.INSTANCE.getField().setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

        telemetry = telemetryFromOpMode;
        currentAlliance = alliance;
        currentDriveMode = DriveMode.FIELD;

        inFarShootingZone = false;
        inNearShootingZone = false;
        isAlignedToGoal = false;

        idleTimer = new ElapsedTime();
        lastRelocalizeTimer = new ElapsedTime();

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
        if (lastKnownPose != null &&
                (
                    lastKnownPose.distanceFrom(this.getPose()) > 0.5
                    || Math.abs(Math.toDegrees(lastKnownPose.getHeading()) - Math.toDegrees(this.getPose().getHeading())) > 0.5
                )
            ) {
            idleTimer.reset();
        }

        lastKnownPose = this.getPose();

        checkForFarShootZone(lastKnownPose.getX(), lastKnownPose.getY(), 12);
        checkForNearShootZone(lastKnownPose.getX(), lastKnownPose.getY(), 12);
        updateHeadingErrorForAutoAimTrig();

        if (Math.abs(Math.toDegrees(lastAutoAimHeadingError)) < 3) {
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
        telemetry.addData("InFarShootingZone", inFarShootingZone);
        telemetry.addData("InNearShootingZone", inNearShootingZone);
        telemetry.addData("IsAlignedToGoal", isAlignedToGoal);
        telemetry.addData("IdleTimer", idleTimer.milliseconds());

        telemetry.addData("LLPedroX", relocalizePedroX);
        telemetry.addData("LLPedroY", relocalizePedroY);
        telemetry.addData("LastRelocalizeTimer", lastRelocalizeTimer.milliseconds());

        if (debug) { drawDebugField(); }
    }

    public void updateLimeLight() {
        double currentHeading = this.follower.getHeading();
        limelight.updateRobotOrientation(Math.toDegrees(currentHeading)+90); // subtract 90 degrees here for pedropathing heading conversion

        if (shouldRelocalize
                && lastRelocalizeTimer.milliseconds() > 5000
                && idleTimer.milliseconds() > 100) {

            // Learned that Tx, Ty, and Ta are degrees of error from tag, not meters.
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();

                if (botpose != null) {
                    // convert from meters to inches and adjust for 0,0 origin like pedropathing instead of -72,-72 that limelight uses
                    relocalizePedroX = (botpose.getPosition().y * 39.3700787) + 72.0; // x and y are intentionally flipped here
                    relocalizePedroY = -(botpose.getPosition().x * 39.3700787) + 72.0;

                    Pose newPedroPose = new Pose(relocalizePedroX, relocalizePedroY, currentHeading);

                    if (Math.abs(botpose.getPosition().z) < 0.1
                            && relocalizePedroX > 0 && relocalizePedroX < 144
                            && relocalizePedroY > 0 && relocalizePedroY < 144) {
                        this.follower.setPose(newPedroPose);
                        lastRelocalizeTimer.reset();
                    }
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
    public void updateHeadingErrorForAutoAimTrig() {
        Pose currentPose = this.getPose();
        Pose goalPose;

        if (currentAlliance == Alliance.RED) {
            goalPose = Poses.RED_GOAL_AIMING_POSE;
        } else {
            goalPose = Poses.BLUE_GOAL_AIMING_POSE;
        }

        // get the "vector" from current robot position to goal position
        double dx = goalPose.getX() - currentPose.getX();
        double dy = goalPose.getY() - currentPose.getY();

        // use atan2 to get heading from x-axis to goal in radians
        double targetHeading = Math.atan2(dy, dx);

        double turnDirection = MathFunctions.getTurnDirection(currentPose.getHeading(), targetHeading);
        double angleDifference = MathFunctions.getSmallestAngleDifference(currentPose.getHeading(), targetHeading);
        double headingError = turnDirection * angleDifference;

        lastAutoAimHeadingError = headingError;
    }

    public double getHeadingErrorForAutoAimTrig() {
        // Use deadband to protect against sign flipping near PI
        if (Math.abs(lastAutoAimHeadingError) < Math.toRadians(1.5)) {
            headingPIDFController.updateError(0);
        } else {
            headingPIDFController.updateError(lastAutoAimHeadingError);
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
                .setGlobalDeceleration()
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
    public void autoRelocalize(boolean allow) {
        shouldRelocalize = allow;
    }

    public void resetHeadingTo90Degrees() {
        Pose newHeadingPose = new Pose(this.getPose().getX(), this.getPose().getY(), Math.toRadians(90));
        this.follower.setPose(newHeadingPose);
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

    private void drawDebugField() {
        Pose pose = follower.getPose();
        FieldManager panelsField = PanelsField.INSTANCE.getField();
        PoseHistory poseHistory = follower.getPoseHistory();

        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        // Draw pose history
        panelsField.setStyle(historyLook);
        if (poseHistory != null) {
            int size = poseHistory.getXPositionsArray().length;
            for (int i = 0; i < size - 1; i++) {
                panelsField.moveCursor(poseHistory.getXPositionsArray()[i], poseHistory.getYPositionsArray()[i]);
                panelsField.line(poseHistory.getXPositionsArray()[i + 1], poseHistory.getYPositionsArray()[i + 1]);
            }
        }

        // Draw robot
        panelsField.setStyle(robotLook);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(9); // ROBOT_RADIUS

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * 9);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(robotLook);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);

        // Draw limelight relocalization position
        if (relocalizePedroX != 0.0 && relocalizePedroY != 0.0) {
            panelsField.setStyle(limelightLook);
            panelsField.moveCursor(relocalizePedroX, relocalizePedroY);
            panelsField.circle(5); // Draw a slightly smaller circle for the limelight pose
        }

        panelsField.update();
    }
    //endregion
}