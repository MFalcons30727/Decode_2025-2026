package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
TODO:
- If we get roadblocked on something, try to pivot to another item if possible
- and we can come back and work on the stuff that doesn't work later.

- New Bot Setup (get us back to where the old bot was)
    - Test all hardware to see what may need to be reversed (test using this teleOp)
        - Flywheel - Done
        - Intake - Done
        - Indexer - Done
        - Hood - Not Working
        - Pinpoint - Done
        - Limelight - Done
    - Remeasure deadwheel offsets - Done
    - Retune PedroPathing constants (use Tuning class under PedroPathing folder) - Done
    - Test PedroPathing poses
    - Add intake to the autonomous state machine - Done
    - Retune flywheel (use FlywheelTuningWithPanels) - Done
- New functionality
    - plug in InterPLUT table values for hood adjustment and velocity scaling
    - Test field-centric driving - Done
    - Test odometry-based auto-aim - Done
    - Test auto-park
    - Fix abort path
    - Add Limelight odometry correction
    - Add Limelight-based auto-aim (if needed) - Done
    - Add ability to start the shooter from anywhere but don't feed artifact until odometry tells
      us we're inside of one of the allowed shooting areas.
    - Figure out a way to pass the final pose of Auto as the start pose of TeleOp
    - LEDs?  Red means red alliance green means blue alliance
    - Toggle slow mode (for precision driving over speed)
    - Alliance swapping in TeleOp - Done
    - Test to see if flywheel is already being given 100% power after shooting
    -   If so, PIDF tuning alone likely can't make it more efficient.
    -   May need to decrease compression
*/

@TeleOp(name = "FieldCentricBlueTeleOp", group = "TeleOp")
public class FieldCentricBlueTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DriverDanny driver;
    private ShooterMcGavin shooter;
    //private double currentTargetVelocity = 1100;
    //private double currentHoodPosition = 0;

    @Override
    public void init() {
        DriverDanny.Alliance startingAlliance = DriverDanny.Alliance.BLUE;
        Pose startingPose = DriverDanny.Poses.BLUE_FAR_START_POSE;

        //if (DriverDanny.currentAlliance != null) {
        //    startingAlliance = DriverDanny.currentAlliance;
        //}

        //if (DriverDanny.lastKnownPose != null) {
        //    startingPose = DriverDanny.lastKnownPose;
        //}

        driver = new DriverDanny(hardwareMap,
                telemetry,
                startingAlliance,
                startingPose);

        shooter = new ShooterMcGavin(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }

    @Override
    public void loop() {
        driver.update();
        //shooter.update();
        shooter.update2(driver.getCurrentDistanceFromGoal());

        double joyY = -gamepad1.left_stick_y; // leaving this inverted so it works for robotCentricDrive (and we adjust for it on fieldCentricDrive)
        double joyX = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (gamepad2.aWasPressed() && !shooter.isShooting()) {
            try {
                //shooter.startShootingFromDistance(driver.getCurrentDistanceFromGoal());
                //shooter.startShootingAtVelocityAndHoodPosition(currentTargetVelocity, currentHoodPosition);
                shooter.startShooting();
            } catch (Exception e) {
                telemetry.addData("shooter", "NOT IN RANGE");
            }
        }

        // auto aim using headingError on field-centric driving. no pedropathing needed.
        if (gamepad2.right_trigger > 0.25 && DriverDanny.currentDriveMode == DriverDanny.DriveMode.FIELD) {
            rotate = driver.getHeadingErrorForAutoAimTrig();
        }

        if (gamepad2.right_bumper && DriverDanny.currentDriveMode == DriverDanny.DriveMode.FIELD) {
            rotate = driver.getHeadingErrorForAutoAimLimelight();

            //if not able to find april tag revert to Trig based aiming
            if (rotate == -1) {
                rotate = driver.getHeadingErrorForAutoAimTrig();
            }
        }

        if (gamepad2.left_trigger > 0.25 && !shooter.isShooting()) {
            shooter.turnOnIntake();
        } else if (!shooter.isShooting()) {
            shooter.turnOffIntake();
        }
        // TODO: fix hood
//        if (gamepad2.dpad_up) {
//            currentHoodPosition += 0.05;
//        }
//
//        if (gamepad2.dpad_down) {
//            currentHoodPosition -= 0.05;
//        }

        // this is for LUT table building
//        if (gamepad2.yWasPressed()) {
//            currentTargetVelocity += 20;
//        }
//
//        if (gamepad2.aWasPressed()) {
//            currentTargetVelocity -= 20;
//        }

        if (gamepad1.yWasPressed()) {
            driver.relocalize();
        }

        if (gamepad1.left_bumper) {
            driver.swapCurrentAlliance(); // lets us swap our alliance (for auto-aim / driver testing)
        }

        if (gamepad1.xWasPressed()) {
            driver.finalPark();
        }

        if (gamepad1.aWasPressed()) {
            driver.abortPath();
        }

        if (gamepad1.bWasPressed()) {
            driver.toggleSlowMode(); // allows us to cut robot movement speed in half when precision is needed
        }

        if (gamepad1.right_bumper) {
            driver.swapCurrentDriveMode();
        }

        // this drive function uses field-centric driving by default unless toggled to robot
        driver.drive(joyY, joyX, rotate);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("VTest", currentTargetVelocity);
        //telemetry.addData("HTest", currentHoodPosition);
        telemetry.update();
    }
}
