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
        EAT_TOP_RED_ARTIFACTS
    }

    // these are all the poses we are going to be in
    private final Pose redFarStartPose = new Pose(48, 8, Math.toRadians(90));
    private final Pose redShootingPose = new Pose (48, 96, Math.toRadians(135));
    private final Pose redTopArtifactsPose = new Pose (41, 84,  Math.toRadians(180));
    private final Pose eatTopRedArtifactsPose = new Pose (19, 84,  Math.toRadians(180));

    private Follower follower; // part of the Pedro Pathing package, follows the path
    private Timer pathTimer, opModeTimer; // this line makes sure you don't use sleeps, this lets the auto do multiple things at once and move smoothly.
    private PathState currentPathState;
    private boolean currentlyShooting = false;

    private PathChain redFarStartToRedShootingPath;
    private PathChain redShootingToRedTopArtifactsPath;
    private PathChain eatTopRedArtifactsPath;

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
                }
            case EAT_TOP_RED_ARTIFACTS:
                if (!follower.isBusy()){
                    follower.followPath(eatTopRedArtifactsPath, true);
                }
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
        follower.setPose(redFarStartPose);
    }

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
        currentlyShooting = true;

        // when done shooting, set currentlyShooting back to false so that the next state can happen
        currentlyShooting = false;
    }

}
