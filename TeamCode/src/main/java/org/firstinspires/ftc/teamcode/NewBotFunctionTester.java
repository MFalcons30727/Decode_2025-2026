package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
CONTROLS:
 - Hold Right Trigger: Turn on flywheel (release to stop)
 - Hold Left Trigger: Turn on intake and indexer (release to stop)
 - Press Dpad Up: Set hood servo position to 1.0 (single press)
 - Press Dpad Down: Set hood servo position to 0.0 (single press)
 */

@TeleOp (name="NewBotFunctionTester", group = "TeleOp")
public class NewBotFunctionTester extends OpMode {
    private DriverDanny driver;
    private ShooterMcGavin shooter;

    private boolean shooting = false;

    @Override
    public void init() {
        driver = new DriverDanny(hardwareMap,
                telemetry,
                DriverDanny.Alliance.BLUE,
                DriverDanny.Poses.BLUE_FAR_START_POSE);

        shooter = new ShooterMcGavin(hardwareMap, telemetry);
        shooter.testMode = true;
    }

    @Override
    public void loop() {
        driver.update();
        shooter.update();

        double joyY = -gamepad1.left_stick_y;
        double joyX = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        driver.robotCentricDrive(joyY, joyX, rotate);

        if (gamepad2.x){
            shooter.turnOnFlywheel();
        }
        if(gamepad2.y){
            shooter.turnOffFlywheel();
        }

        if (gamepad2.left_trigger > 0.25 || gamepad2.right_trigger > 0.25)
        {
            shooter.turnOnIntake();
        } else {
            shooter.turnOffIntake();
        }

        if (gamepad2.left_trigger > 0.25) {
            shooter.turnOnIndexer();
        } else {
            shooter.turnOffIndexer();
        }

        if (gamepad2.dpad_up) {
            shooter.setHoodUp();
        }

        if (gamepad2.dpad_down) {
            shooter.setHoodDown();
        }

        telemetry.update();
    }
}
