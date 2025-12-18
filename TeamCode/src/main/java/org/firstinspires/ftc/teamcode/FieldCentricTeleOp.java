package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="TeleOp", group = "TeleOp")
public class FieldCentricTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DriverDanny driver;
    private ShooterMcGavin shooter;
    // private DcMotor intake = null;

    @Override
    public void init() {
        driver = new DriverDanny(hardwareMap,
                                    telemetry,
                                    DriverDanny.Poses.RED_FAR_START_POSE,
                                    DriverDanny.Team.RED);
        //intake = hardwareMap.get(DcMotor.class, "intake");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        runtime.reset();
    }

    @Override
    public void loop() {
        driver.update();
        shooter.update();

        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        driver.fieldCentricDrive(forward, strafe, rotate);

        if (gamepad2.right_trigger > 0) {
            shooter.startShootingAtVelocity(1100);
        }

        if (gamepad2.left_trigger > 0) {
            shooter.startShootingAtVelocity(1400);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}


