package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="FieldCentricBlueTeleOp", group = "TeleOp")
public class FieldCentricBlueTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DriverDanny driver;
    private ShooterMcGavin shooter;
    // private DcMotor intake = null;

    @Override
    public void init() {
        // not sure if this is going to work because the starting pose is going to be different
        // need to find a way to get what our starting pose is in TeleOp
        driver = new DriverDanny(hardwareMap,
                telemetry,
                DriverDanny.Alliance.BLUE,
                DriverDanny.Poses.BLUE_FAR_START_POSE);
        shooter = new ShooterMcGavin(hardwareMap, telemetry);
        //intake = hardwareMap.get(DcMotor.class, "intake");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }

    @Override
    public void loop() {
        driver.update();
        shooter.update();

        double forward = gamepad1.left_stick_x;
        double strafe = gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;


        if ((gamepad1.right_trigger > 0.25 || gamepad2.right_trigger > 0.25)
                && !shooter.isShooting()) {
            //shooter.startShootingAtVelocity(1100); // can go back to using this if needed until we have velocity scaling working
            try {
                shooter.startShootingFromDistance(driver.getCurrentDistanceFromGoal());
            } catch (Exception e) {
                telemetry.addData("shooter", "NOT IN RANGE");
            }

        }


        // auto aim using headingError on field-centric driving.  no pedropathing needed.
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            rotate = driver.getHeadingErrorForAutoAim();
        }

        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
            driver.swapCurrentAlliance(); // lets us swap our alliance (for auto-aim testing)
        }

        if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
            driver.finalPark();
        }

        if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
            driver.abortPath();
        }


        // if holding down right bumper, it will lock the heading with autoaim.
        // otherwise, it will use the rotation from the right stick x
        driver.fieldCentricDrive(forward, strafe, rotate);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}


