package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
TODO:
- Retune PedroPathing constants (use Tuning class under PedroPathing folder)
- Test auto-park
- Test PedroPathing poses
- Add Limelight odometry correction
- Add ability to start the shooter from anywhere but don't feed artifact until odometry tells
      us we're inside of one of the allowed shooting areas.
- Test passing the final pose of Auto as the start pose of TeleOp

- New shooter button that:
  - Starts flywheel
  - Starts Auto-aim
  - Waits to be still for 0.5 seconds
  - Only shoots if within shooting zone
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
        shooter.updateWithLUT(driver.getCurrentDistanceFromGoal());

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
