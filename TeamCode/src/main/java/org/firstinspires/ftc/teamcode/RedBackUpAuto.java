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
public class RedBackUpAuto extends OpMode {
    // A state machine keeps track of what state or step is currently running
    public enum PathState {
        MOVE_FROM_START_POS_TO_SHOOTING_LINE,
        SHOOT_PRELOAD,
        MOVE_FROM_SHOOTING_LINE_TO_RED_GATE,
        DONE
    }

    // these are all the poses we are going to be in
    private final Pose redBackupStartPose = new Pose(122, 125, Math.toRadians(45));
    private final Pose redShootingPose = new Pose (96, 96, Math.toRadians(45));
    private final Pose parkRedGatePose = new Pose (120,70, Math.toRadians(180));



    private Follower follower; // part of the Pedro Pathing package, follows the path
    private Timer pathTimer, opModeTimer; // this line makes sure you don't use sleeps, this lets the auto do multiple things at once and move smoothly.
    private PathState currentPathState;
    private ShooterMcGavin shooter;

    private PathChain redBackupStartToRedShootingPath;
    private PathChain parkAtRedGatePath;



    public void buildPaths(){
        redBackupStartToRedShootingPath = follower.pathBuilder()
                .addPath(new BezierLine(redBackupStartPose, redShootingPose))
                .setLinearHeadingInterpolation(redBackupStartPose.getHeading(), redShootingPose.getHeading())
                .build();
        parkAtRedGatePath = follower.pathBuilder()
                .addPath(new BezierLine(redShootingPose, parkRedGatePose))
                .setLinearHeadingInterpolation(redShootingPose.getHeading(), parkRedGatePose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch (currentPathState){
            case MOVE_FROM_START_POS_TO_SHOOTING_LINE:
                follower.followPath(redBackupStartToRedShootingPath, true);
                setPathState(PathState.SHOOT_PRELOAD); // reset timer, new state
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    shooter.startShooting();
                    setPathState(PathState.MOVE_FROM_SHOOTING_LINE_TO_RED_GATE);
                }
                break;
            case MOVE_FROM_SHOOTING_LINE_TO_RED_GATE:
                if (!follower.isBusy() && !shooter.isShooting()){
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
        shooter = new ShooterMcGavin(hardwareMap, telemetry);
        // TODO add any other init stuff like flywheels or limelight

        buildPaths();
        follower.setStartingPose(redBackupStartPose);
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