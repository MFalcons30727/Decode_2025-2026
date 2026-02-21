package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/* TODO - Denham Practices
Programming
1) Re-measure LUT tables
2) Re-tune Pedro Pathing
2) Test adjusted trig goal pose (by unplugging the limelight)
3) Test heading lock button
4) New Autos (don't forget to open gate and collect the 3 far ones we haven't yet!)
 */
@TeleOp(name = "ImprovedTeleOp", group = "TeleOp")
public class  ImprovedTeleOp extends OpMode {
    //NEED TO REMEMBER THAT SETTING THIS IS TRUE IS WHAT ENABLES US TO CARRY OVER LAST POSE FROM AUTO.
    private static final boolean USE_LAST_POSE_FROM_AUTO = true;
    private ElapsedTime runtime = new ElapsedTime();
    private DriverDanny driver;
    private ShooterMcGavin shooter;
    private boolean headingLock = false;

    @Override
    public void init() {
        DriverDanny.Alliance startingAlliance = DriverDanny.Alliance.BLUE;
        Pose startingPose = DriverDanny.Poses.BFA_START_POSE;
        //Pose startingPose = DriverDanny.Poses.TEST_START_POSE;

        if (USE_LAST_POSE_FROM_AUTO && DriverDanny.currentAlliance != null) {
            startingAlliance = DriverDanny.currentAlliance;
        }

        if (USE_LAST_POSE_FROM_AUTO && DriverDanny.lastKnownPose != null) {
            startingPose = DriverDanny.lastKnownPose;
        }

        driver = new DriverDanny(hardwareMap,
                telemetry,
                startingAlliance,
                startingPose);

        driver.autoRelocalize(true);

        shooter = new ShooterMcGavin(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }

    @Override
    public void loop() {
        driver.update();
        shooter.updateWithLUT(driver.getCurrentDistanceFromGoal());

        // we only need to allow locking our heading and allowing restrictedShooting when shooting routine has been started
        if(!shooter.isShooting()) {
            headingLock = false;
        }

        double joyY = -gamepad1.left_stick_y; // leaving this inverted so it works for robotCentricDrive (and we adjust for it on fieldCentricDrive)
        double joyX = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        //region Gamepad1
        if (gamepad1.dpadUpWasPressed() || gamepad1.dpadDownWasPressed()) {
            driver.swapCurrentAlliance(); // lets us swap our alliance (for auto-aim / driver testing)
        }

        if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
            driver.swapCurrentDriveMode();
        }

//        if (gamepad1.aWasPressed()) {
//            driver.abortPath();
//        }

        if (gamepad1.bWasPressed()) {
            driver.toggleSlowMode(); // allows us to cut robot movement speed in half when precision is needed
        }

//        if (gamepad1.yWasPressed()) {
//            shooter.activateKickstand();
//        }

//        if (gamepad1.xWasPressed()) {
//            driver.finalPark();
//        }

       if (gamepad1.yWasPressed()) {
           driver.autoRelocalize(false);
       }
        //endregion

        //region Gamepad2
        if (gamepad2.aWasPressed() && !shooter.isShooting()) {
            try {
                shooter.startShooting();
            } catch (Exception e) {
                telemetry.addData("shooter", "NOT IN RANGE");
            }
        }

        if (gamepad2.bWasPressed() && !shooter.isShooting()) {
            headingLock = true;
            ShooterMcGavin.restrictedShooting = true;

            try {
                shooter.startShooting();
            } catch (Exception e) {
                telemetry.addData("shooter", "NOT IN RANGE");
            }
        }

        if (gamepad2.xWasPressed() && shooter.isShooting()) {
            shooter.stopShooting();
            headingLock = false;
            ShooterMcGavin.restrictedShooting = false;
        }

        if ((headingLock || gamepad2.right_bumper || gamepad2.right_trigger > 0.25) && DriverDanny.currentDriveMode == DriverDanny.DriveMode.FIELD) {
            //rotate = driver.getHeadingErrorForAutoAimLimelight();

            //if not able to find april tag revert to Trig based aiming
            //if (rotate == -1) {
                rotate = driver.getHeadingErrorForAutoAimTrig();
            //}
        }
//        else {
//            // TODO: Turn off auto-aim LED :)
//        }

        if (gamepad2.left_trigger > 0.25 && !shooter.isShooting()) {
            shooter.turnOnIntake();
        } else if (!shooter.isShooting()) {
            shooter.turnOffIntake();
        }

        if (gamepad2.left_bumper && !shooter.isShooting()) {
            shooter.reverseIntake();
        }
        //endregion

        // this drive function uses field-centric driving by default unless toggled to robot
        driver.drive(joyY, joyX, rotate);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("HeadingLock", headingLock);
        telemetry.update();
    }
}