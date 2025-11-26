package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// to access the panels dashboard it should be our robot's ip address and port 8001
// just need to open it in the browser like this: 192.168.43.1:8001
// https://pedropathing.com/docs/pathing/dashboard

@Autonomous
public class PedroPathingRedFarAuto extends OpMode {
    // A state machine keeps track of what state or step is currently running
    public enum PathState {
        MOVE_FROM_START_POS_TO_SHOOTING_LINE,
        SHOOT_PRELOAD,
        MOVE_FROM_SHOOTING_LINE_TO_TOP_RED_ARTIFACTS,
        EAT_TOP_RED_ARTIFACTS,
        MOVE_FROM_TOP_RED_ARTIFACTS_TO_SHOOTING_LINE,
        SHOOT_TOP_RED_ARTIFACTS,
        MOVE_FROM_SHOOTING_LINE_TO_MIDDLE_RED_ARTIFACTS,
        EAT_MIDDLE_RED_ARTIFACTS,
        MOVE_FROM_MIDDLE_RED_ARTIFACTS_TO_SHOOTING_LINE,
        SHOOT_MIDDLE_RED_ARTIFACTS,
        MOVE_FROM_SHOOTING_LINE_TO_BOTTOM_RED_ARTIFACTS,
        EAT_BOTTOM_RED_ARTIFACTS,
        MOVE_FROM_BOTTOM_RED_ARTIFACTS_TO_SHOOTING_LINE,
        SHOOT_BOTTOM_RED_ARTIFACTS,
        MOVE_FROM_SHOOTING_LINE_TO_RED_GATE,
        DONE
    }

    // these are all the poses we are going to be in
    private final Pose redFarStartPose = new Pose(48, 8, Math.toRadians(90));
    private final Pose redShootingPose = new Pose (48, 96, Math.toRadians(135));
    private final Pose redTopArtifactsPose = new Pose (41, 84,  Math.toRadians(180));
    private final Pose eatTopRedArtifactsPose = new Pose (20, 84,  Math.toRadians(180));
    private final Pose redMiddleArtifactsPose = new Pose (42, 60,  Math.toRadians(180));
    private final Pose eatMiddleRedArtifactsPose = new Pose (19, 60,  Math.toRadians(180));
    private final Pose redBottomArtifactsPose = new Pose (42, 36, Math.toRadians(180));
    private final Pose eatBottomRedArtifactsPose = new Pose (19,36, Math.toRadians(180));
    private final Pose parkRedGatePose = new Pose (24,71, Math.toRadians(0));



    private Follower follower; // part of the Pedro Pathing package, follows the path
    private Timer pathTimer, opModeTimer; // this line makes sure you don't use sleeps, this lets the auto do multiple things at once and move smoothly.
    private PathState currentPathState;
    private boolean currentlyShooting = false;

    private PathChain redFarStartToRedShootingPath;
    private PathChain redShootingToRedTopArtifactsPath;
    private PathChain eatTopRedArtifactsPath;
    private PathChain returnToRedShootingLineFromRedTopArtifactsPath;
    private PathChain redShootingToRedMiddleArtifactsPath;
    private PathChain eatMiddleRedArtifactsPath;
    private PathChain returnToRedShootingLineFromRedMiddleArtifactsPath;
    private PathChain redShootingToRedBottomArtifactsPath;
    private PathChain eatBottomRedArtifactsPath;
    private PathChain returnToRedShootingLineFromRedBottomArtifactsPath;
    private PathChain parkAtRedGatePath;



    public void buildPaths(){
        redFarStartToRedShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(redFarStartPose, redShootingPose))
                .setLinearHeadingInterpolation(redFarStartPose.getHeading(), redShootingPose.getHeading())
                .build();
        redShootingToRedTopArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(redShootingPose, redTopArtifactsPose))
                .setLinearHeadingInterpolation(redShootingPose.getHeading(), redTopArtifactsPose.getHeading())
                .build();
        eatTopRedArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(redTopArtifactsPose, eatTopRedArtifactsPose))
                .setLinearHeadingInterpolation(redTopArtifactsPose.getHeading(), eatTopRedArtifactsPose.getHeading())
                .build();
        returnToRedShootingLineFromRedTopArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(eatTopRedArtifactsPose, redShootingPose))
                .setLinearHeadingInterpolation(eatTopRedArtifactsPose.getHeading(), redShootingPose.getHeading())
                .build();
        redShootingToRedMiddleArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(redShootingPose, redMiddleArtifactsPose))
                .setLinearHeadingInterpolation(redShootingPose.getHeading(), redMiddleArtifactsPose.getHeading())
                .build();
        eatMiddleRedArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(redMiddleArtifactsPose, eatMiddleRedArtifactsPose))
                .setLinearHeadingInterpolation(redMiddleArtifactsPose.getHeading(), eatMiddleRedArtifactsPose.getHeading())
                .build();
        returnToRedShootingLineFromRedMiddleArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(eatMiddleRedArtifactsPose, redShootingPose))
                .setLinearHeadingInterpolation(eatMiddleRedArtifactsPose.getHeading(), redShootingPose.getHeading())
                .build();
        redShootingToRedBottomArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(redShootingPose, redBottomArtifactsPose))
                .setLinearHeadingInterpolation(redShootingPose.getHeading(), redBottomArtifactsPose.getHeading())
                .build();
        eatBottomRedArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(redBottomArtifactsPose, eatBottomRedArtifactsPose))
                .setLinearHeadingInterpolation(redBottomArtifactsPose.getHeading(), eatBottomRedArtifactsPose.getHeading())
                .build();
        returnToRedShootingLineFromRedBottomArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(eatBottomRedArtifactsPose, redShootingPose))
                .setLinearHeadingInterpolation(eatBottomRedArtifactsPose.getHeading(), redShootingPose.getHeading())
                .build();
        parkAtRedGatePath = follower.pathBuilder()
                .addPath(new BezierLine(redShootingPose, parkRedGatePose))
                .setLinearHeadingInterpolation(redShootingPose.getHeading(), parkRedGatePose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (currentPathState){
            case MOVE_FROM_START_POS_TO_SHOOTING_LINE:
                follower.followPath(redFarStartToRedShootingPath, true);
                setPathState(PathState.SHOOT_PRELOAD); // reset timer, new state
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    shoot(); // in our shoot() function, we should set currentlyShooting to true until all 3 shots are done
                    setPathState(PathState.MOVE_FROM_SHOOTING_LINE_TO_TOP_RED_ARTIFACTS);
                }
                break;
            case MOVE_FROM_SHOOTING_LINE_TO_TOP_RED_ARTIFACTS:
                if (!follower.isBusy() && !currentlyShooting){
                    follower.followPath(redShootingToRedTopArtifactsPath, true);
                    setPathState(PathState.EAT_TOP_RED_ARTIFACTS);
                }
                break;
            case EAT_TOP_RED_ARTIFACTS:
                if (!follower.isBusy()){
                    follower.followPath(eatTopRedArtifactsPath, true);
                    setPathState(PathState.MOVE_FROM_TOP_RED_ARTIFACTS_TO_SHOOTING_LINE);
                }
                break;
            case MOVE_FROM_TOP_RED_ARTIFACTS_TO_SHOOTING_LINE:
                if (!follower.isBusy()){
                    follower.followPath(returnToRedShootingLineFromRedTopArtifactsPath, true);
                    setPathState(PathState.SHOOT_TOP_RED_ARTIFACTS);
                }
                break;
            case SHOOT_TOP_RED_ARTIFACTS:
                if (!follower.isBusy()) {
                    shoot(); // in our shoot() function, we should set currentlyShooting to true until all 3 shots are done
                    setPathState(PathState.MOVE_FROM_SHOOTING_LINE_TO_MIDDLE_RED_ARTIFACTS);
                }
                break;
            case MOVE_FROM_SHOOTING_LINE_TO_MIDDLE_RED_ARTIFACTS:
                if (!follower.isBusy() && !currentlyShooting){
                    follower.followPath(redShootingToRedMiddleArtifactsPath, true);
                    setPathState(PathState.EAT_MIDDLE_RED_ARTIFACTS);
                }
                break;
            case EAT_MIDDLE_RED_ARTIFACTS:
                if (!follower.isBusy()){
                    follower.followPath(eatMiddleRedArtifactsPath, true);
                    setPathState(PathState.MOVE_FROM_MIDDLE_RED_ARTIFACTS_TO_SHOOTING_LINE);
                }
                break;
            case MOVE_FROM_MIDDLE_RED_ARTIFACTS_TO_SHOOTING_LINE:
                if (!follower.isBusy()){
                    follower.followPath(returnToRedShootingLineFromRedMiddleArtifactsPath, true);
                    setPathState(PathState.SHOOT_MIDDLE_RED_ARTIFACTS);
                }
            case SHOOT_MIDDLE_RED_ARTIFACTS:
                if (!follower.isBusy()) {
                    shoot(); // in our shoot() function, we should set currentlyShooting to true until all 3 shots are done
                    setPathState(PathState.MOVE_FROM_SHOOTING_LINE_TO_BOTTOM_RED_ARTIFACTS);
                }
                break;
            case MOVE_FROM_SHOOTING_LINE_TO_BOTTOM_RED_ARTIFACTS:
                if (!follower.isBusy() && !currentlyShooting){
                    follower.followPath(redShootingToRedBottomArtifactsPath, true);
                    setPathState(PathState.EAT_BOTTOM_RED_ARTIFACTS);
                }
                break;
            case EAT_BOTTOM_RED_ARTIFACTS:
                if (!follower.isBusy()){
                    follower.followPath(eatBottomRedArtifactsPath, true);
                    setPathState(PathState.MOVE_FROM_BOTTOM_RED_ARTIFACTS_TO_SHOOTING_LINE);
                }
                break;
            case MOVE_FROM_BOTTOM_RED_ARTIFACTS_TO_SHOOTING_LINE:
                if (!follower.isBusy()){
                    follower.followPath(returnToRedShootingLineFromRedBottomArtifactsPath, true);
                    setPathState(PathState.SHOOT_BOTTOM_RED_ARTIFACTS);
                }
                break;
            case SHOOT_BOTTOM_RED_ARTIFACTS:
                if (!follower.isBusy()) {
                    shoot(); // in our shoot() function, we should set currentlyShooting to true until all 3 shots are done
                    setPathState(PathState.MOVE_FROM_SHOOTING_LINE_TO_RED_GATE);
                }
                break;
            case MOVE_FROM_SHOOTING_LINE_TO_RED_GATE:
                if (!follower.isBusy() && !currentlyShooting){
                    follower.followPath(parkAtRedGatePath, true);
                    setPathState(PathState.DONE);
                }
                break;
            default:
                telemetry.addLine("no state right now");
                break;
        }
    }

    // this is to help transition states
    public void setPathState(PathState newState) {
        currentPathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {

        // this is the state we are always starting out with.
        // also I think the timers keep track of how long the Opmode has been running for

        currentPathState = PathState.MOVE_FROM_START_POS_TO_SHOOTING_LINE;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add any other init stuff like flywheels or limelight

        buildPaths();
        follower.setStartingPose(redFarStartPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(currentPathState);
    }

    @Override
    public void loop(){
        follower.update(); // YOU NEED THIS UPDATE LINE
        statePathUpdate();

        telemetry.addData("path state", currentPathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("x", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }

    public void shoot() {
        // when done shooting, set currentlyShooting back to false so that the next state can happen
        // for now, just wait 10 seconds before continuing to next step
        if (pathTimer.getElapsedTimeSeconds() < 3) { currentlyShooting = true; }
        else { currentlyShooting = false; }
    }

}