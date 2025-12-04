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
public class PedroPathingBlueFarAuto extends OpMode {
    // A state machine keeps track of what state or step is currently running
    public enum PathState {
        MOVE_FROM_START_POS_TO_SHOOTING_LINE,
        SHOOT_PRELOAD,
        MOVE_FROM_SHOOTING_LINE_TO_TOP_BLUE_ARTIFACTS,
        EAT_TOP_BLUE_ARTIFACTS,
        MOVE_FROM_TOP_BLUE_ARTIFACTS_TO_SHOOTING_LINE,
        SHOOT_TOP_BLUE_ARTIFACTS,
        MOVE_FROM_SHOOTING_LINE_TO_MIDDLE_BLUE_ARTIFACTS,
        EAT_MIDDLE_BLUE_ARTIFACTS,
        MOVE_FROM_MIDDLE_BLUE_ARTIFACTS_TO_SHOOTING_LINE,
        SHOOT_MIDDLE_BLUE_ARTIFACTS,
        MOVE_FROM_SHOOTING_LINE_TO_BOTTOM_BLUE_ARTIFACTS,
        EAT_BOTTOM_BLUE_ARTIFACTS,
        MOVE_FROM_BOTTOM_BLUE_ARTIFACTS_TO_SHOOTING_LINE,
        SHOOT_BOTTOM_BLUE_ARTIFACTS,
        MOVE_FROM_SHOOTING_LINE_TO_BLUE_GATE,
        DONE
    }

    // these are all the poses we are going to be in
    private final Pose blueFarStartPose = new Pose(48, 8, Math.toRadians(90));
    private final Pose blueShootingPose = new Pose (48, 96, Math.toRadians(135));
    private final Pose blueTopArtifactsPose = new Pose (41, 84,  Math.toRadians(180));
    private final Pose eatTopBlueArtifactsPose = new Pose (20, 84,  Math.toRadians(180));
    private final Pose blueMiddleArtifactsPose = new Pose (42, 60,  Math.toRadians(180));
    private final Pose eatMiddleBlueArtifactsPose = new Pose (19, 60,  Math.toRadians(180));
    private final Pose blueBottomArtifactsPose = new Pose (42, 36, Math.toRadians(180));
    private final Pose eatBottomBlueArtifactsPose = new Pose (19,36, Math.toRadians(180));
    private final Pose parkBlueGatePose = new Pose (24,71, Math.toRadians(0));



    private Follower follower; // part of the Pedro Pathing package, follows the path
    private Timer pathTimer, opModeTimer; // this line makes sure you don't use sleeps, this lets the auto do multiple things at once and move smoothly.
    private PathState currentPathState;
    private ShooterMcGavin shooter;

    private PathChain blueFarStartToBlueShootingPath;
    private PathChain blueShootingToBlueTopArtifactsPath;
    private PathChain eatTopBlueArtifactsPath;
    private PathChain returnToBlueShootingLineFromBlueTopArtifactsPath;
    private PathChain blueShootingToBlueMiddleArtifactsPath;
    private PathChain eatMiddleBlueArtifactsPath;
    private PathChain returnToBlueShootingLineFromBlueMiddleArtifactsPath;
    private PathChain blueShootingToBlueBottomArtifactsPath;
    private PathChain eatBottomBlueArtifactsPath;
    private PathChain returnToBlueShootingLineFromBlueBottomArtifactsPath;
    private PathChain parkAtBlueGatePath;



    public void buildPaths(){
        blueFarStartToBlueShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(blueFarStartPose, blueShootingPose))
                .setLinearHeadingInterpolation(blueFarStartPose.getHeading(), blueShootingPose.getHeading())
                .build();
        blueShootingToBlueTopArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(blueShootingPose, blueTopArtifactsPose))
                .setLinearHeadingInterpolation(blueShootingPose.getHeading(), blueTopArtifactsPose.getHeading())
                .build();
        eatTopBlueArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(blueTopArtifactsPose, eatTopBlueArtifactsPose))
                .setLinearHeadingInterpolation(blueTopArtifactsPose.getHeading(), eatTopBlueArtifactsPose.getHeading())
                .build();
        returnToBlueShootingLineFromBlueTopArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(eatTopBlueArtifactsPose, blueShootingPose))
                .setLinearHeadingInterpolation(eatTopBlueArtifactsPose.getHeading(), blueShootingPose.getHeading())
                .build();
        blueShootingToBlueMiddleArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(blueShootingPose, blueMiddleArtifactsPose))
                .setLinearHeadingInterpolation(blueShootingPose.getHeading(), blueMiddleArtifactsPose.getHeading())
                .build();
        eatMiddleBlueArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(blueMiddleArtifactsPose, eatMiddleBlueArtifactsPose))
                .setLinearHeadingInterpolation(blueMiddleArtifactsPose.getHeading(), eatMiddleBlueArtifactsPose.getHeading())
                .build();
        returnToBlueShootingLineFromBlueMiddleArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(eatMiddleBlueArtifactsPose, blueShootingPose))
                .setLinearHeadingInterpolation(eatMiddleBlueArtifactsPose.getHeading(), blueShootingPose.getHeading())
                .build();
        blueShootingToBlueBottomArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(blueShootingPose, blueBottomArtifactsPose))
                .setLinearHeadingInterpolation(blueShootingPose.getHeading(), blueBottomArtifactsPose.getHeading())
                .build();
        eatBottomBlueArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(blueBottomArtifactsPose, eatBottomBlueArtifactsPose))
                .setLinearHeadingInterpolation(blueBottomArtifactsPose.getHeading(), eatBottomBlueArtifactsPose.getHeading())
                .build();
        returnToBlueShootingLineFromBlueBottomArtifactsPath = follower.pathBuilder()
                .addPath(new BezierLine(eatBottomBlueArtifactsPose, blueShootingPose))
                .setLinearHeadingInterpolation(eatBottomBlueArtifactsPose.getHeading(), blueShootingPose.getHeading())
                .build();
        parkAtBlueGatePath = follower.pathBuilder()
                .addPath(new BezierLine(blueShootingPose, parkBlueGatePose))
                .setLinearHeadingInterpolation(blueShootingPose.getHeading(), parkBlueGatePose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (currentPathState){
            case MOVE_FROM_START_POS_TO_SHOOTING_LINE:
                follower.followPath(blueFarStartToBlueShootingPath, true);
                setPathState(PathState.SHOOT_PRELOAD); // reset timer, new state
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    shooter.startShooting();
                    setPathState(PathState.MOVE_FROM_SHOOTING_LINE_TO_TOP_BLUE_ARTIFACTS);
                }
                break;
            case MOVE_FROM_SHOOTING_LINE_TO_TOP_BLUE_ARTIFACTS:
                if (!follower.isBusy() && !shooter.isShooting() && pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(blueShootingToBlueTopArtifactsPath, true);
                    setPathState(PathState.EAT_TOP_BLUE_ARTIFACTS);
                }
                break;
            case EAT_TOP_BLUE_ARTIFACTS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(eatTopBlueArtifactsPath, true);
                    setPathState(PathState.MOVE_FROM_TOP_BLUE_ARTIFACTS_TO_SHOOTING_LINE);
                }
                break;
            case MOVE_FROM_TOP_BLUE_ARTIFACTS_TO_SHOOTING_LINE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(returnToBlueShootingLineFromBlueTopArtifactsPath, true);
                    setPathState(PathState.SHOOT_TOP_BLUE_ARTIFACTS);
                }
                break;
            case SHOOT_TOP_BLUE_ARTIFACTS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    shooter.startShooting();
                    setPathState(PathState.MOVE_FROM_SHOOTING_LINE_TO_MIDDLE_BLUE_ARTIFACTS);
                }
                break;
            case MOVE_FROM_SHOOTING_LINE_TO_MIDDLE_BLUE_ARTIFACTS:
                if (!follower.isBusy() && !shooter.isShooting() && pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(blueShootingToBlueMiddleArtifactsPath, true);
                    setPathState(PathState.EAT_MIDDLE_BLUE_ARTIFACTS);
                }
                break;
            case EAT_MIDDLE_BLUE_ARTIFACTS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(eatMiddleBlueArtifactsPath, true);
                    setPathState(PathState.MOVE_FROM_MIDDLE_BLUE_ARTIFACTS_TO_SHOOTING_LINE);
                }
                break;
            case MOVE_FROM_MIDDLE_BLUE_ARTIFACTS_TO_SHOOTING_LINE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(returnToBlueShootingLineFromBlueMiddleArtifactsPath, true);
                    setPathState(PathState.SHOOT_MIDDLE_BLUE_ARTIFACTS);
                }
            case SHOOT_MIDDLE_BLUE_ARTIFACTS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    shooter.startShooting(); // in our shoot() function, we should set currentlyShooting to true until all 3 shots are done
                    setPathState(PathState.MOVE_FROM_SHOOTING_LINE_TO_BOTTOM_BLUE_ARTIFACTS);
                }
                break;
            case MOVE_FROM_SHOOTING_LINE_TO_BOTTOM_BLUE_ARTIFACTS:
                if (!follower.isBusy() && !shooter.isShooting() && pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(blueShootingToBlueBottomArtifactsPath, true);
                    setPathState(PathState.EAT_BOTTOM_BLUE_ARTIFACTS);
                }
                break;
            case EAT_BOTTOM_BLUE_ARTIFACTS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(eatBottomBlueArtifactsPath, true);
                    setPathState(PathState.MOVE_FROM_BOTTOM_BLUE_ARTIFACTS_TO_SHOOTING_LINE);
                }
                break;
            case MOVE_FROM_BOTTOM_BLUE_ARTIFACTS_TO_SHOOTING_LINE:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(returnToBlueShootingLineFromBlueBottomArtifactsPath, true);
                    setPathState(PathState.SHOOT_BOTTOM_BLUE_ARTIFACTS);
                }
                break;
            case SHOOT_BOTTOM_BLUE_ARTIFACTS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    shooter.startShooting(); // in our shoot() function, we should set currentlyShooting to true until all 3 shots are done
                    setPathState(PathState.MOVE_FROM_SHOOTING_LINE_TO_BLUE_GATE);
                }
                break;
            case MOVE_FROM_SHOOTING_LINE_TO_BLUE_GATE:
                if (!follower.isBusy() && !shooter.isShooting() && pathTimer.getElapsedTimeSeconds() > 1){
                    follower.followPath(parkAtBlueGatePath, true);
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
        shooter = new ShooterMcGavin(hardwareMap);
        // TODO add any other init stuff like flywheels or limelight

        buildPaths();
        follower.setStartingPose(blueFarStartPose);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(currentPathState);
    }

    @Override
    public void loop(){
        follower.update(); // YOU NEED THIS UPDATE LINE
        shooter.update();
        statePathUpdate();

        telemetry.addData("path state", currentPathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("x", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }

}