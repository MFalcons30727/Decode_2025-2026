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
                DriverDanny.Team.BLUE,
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

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (gamepad2.right_trigger > 0) {
            //shooter.startShootingAtVelocity(1100); // can go back to using this if needed until we have velocity scaling working
            shooter.startShootingFromDistance(driver.getCurrentDistanceFromGoal());
        }

        // auto aim using headingError on field-centric driving.  no pedropathing needed.
        if(gamepad2.right_bumper) {
            rotate = driver.getHeadingErrorForAutoAim();
        }

        // if holding down right bumper, it will lock the heading with autoaim.
        // otherwise, it will use the rotation from the right stick x
        driver.fieldCentricDrive(forward, strafe, rotate);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}


