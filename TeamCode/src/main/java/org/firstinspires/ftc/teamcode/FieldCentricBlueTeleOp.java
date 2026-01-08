package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
TODO:
- If we get roadblocked on something, try to pivot to another item if possible
- and we can come back and work on the stuff that doesn't work later.

- We should focus on making an OpMode to handle each of these items so we can work on them
- one at a time.

- New Bot Setup (get us back to where the old bot was)
    - Test all hardware to see what may need to be reversed (test using this teleOp)
        - Flywheel
        - Intake
        - Indexer
        - Hood
        - Pinpoint
        - Limelight
    - Remeasure deadwheel offsets
    - Retune PedroPathing constants (use Tuning class under PedroPathing folder)
    - Test PedroPathing poses (make an opmode for testing this)
    - Retune flywheel (use FlywheelTuningWithPanels)
- New functionality
    - Test field-centric driving
    - Test odometry-based auto-aim
    - Test auto-park
    - Fix abort path
    - Add Limelight odometry correction
    - Add Limelight-based auto-aim (if needed)
    - Add ability to start the shooter from anywhere but don't feed artifact until odometry tells
      us we're inside of one of the allowed shooting areas.
    - Figure out a way to pass the final pose of Auto as the start pose of TeleOp
    - LEDs?
*/

@TeleOp (name="FieldCentricBlueTeleOp", group = "TeleOp")
public class FieldCentricBlueTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
//    private DriverDanny driver;
    private ShooterMcGavin shooter;
private DcMotorEx shootMotor;
    private DcMotor indexer, intake;

    @Override
    public void init() {
        shootMotor = hardwareMap.get(DcMotorEx.class, "shooter");

        indexer = hardwareMap.get(DcMotor.class, "indexer");

        shootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootMotor.setDirection(DcMotorEx.Direction.REVERSE);

        indexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // not sure if this is going to work because the starting pose is going to be different
        // need to find a way to get what our starting pose is in TeleOp
//        driver = new DriverDanny(hardwareMap,
//                telemetry,
//                DriverDanny.Alliance.BLUE,
//                DriverDanny.Poses.BLUE_FAR_START_POSE);
        shooter = new ShooterMcGavin(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }

    @Override
    public void loop() {
//        driver.update();
        //shooter.update();

        double joyY = -gamepad1.left_stick_y; // leaving this inverted so it works for robotCentricDrive (and we adjust for it on fieldCentricDrive)
        double joyX = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        //if (gamepad2.right_trigger > 0.25 && !shooter.isShooting()) {
        if (gamepad2.right_trigger > 0.25) {
            indexer.setPower(1);
            //shooter.startShootingAtVelocity(1100); // can go back to using this if needed until we have velocity scaling working
            shootMotor.setPower(1);
//            try {
//                shooter.startShootingFromDistance(driver.getCurrentDistanceFromGoal());
//            } catch (Exception e) {
//                telemetry.addData("shooter", "NOT IN RANGE");
//            }
        } else {
            indexer.setPower(0);
            shootMotor.setPower(0);
        }

        // auto aim using headingError on field-centric driving.  no pedropathing needed.
//        if (gamepad2.right_bumper) {
//            rotate = driver.getHeadingErrorForAutoAim();
//        }

        // Feed should also run intake

        if (gamepad2.left_trigger > 0.25) {
            shooter.turnOnIntake();
        } else {
            shooter.turnOffIntake();
        }

        if (gamepad2.dpad_up) {
            shooter.setHoodUp();
        }

        if (gamepad2.dpad_down) {
            shooter.setHoodDown();
        }

//        if (gamepad1.aWasPressed()) {
//            driver.swapCurrentAlliance(); // lets us swap our alliance (for auto-aim / driver testing)
//        }
//
//        if (gamepad1.bWasPressed()) {
//            driver.finalPark();
//        }
//
//        if (gamepad1.xWasPressed()) {
//            driver.abortPath();
//        }
//
//
//        // if holding down right bumper, it will lock the heading with autoaim.
//        // otherwise, it will use the rotation from the right stick x
//        driver.fieldCentricDrive(joyY, joyX, rotate);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

    }

}


