package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "LUTSetup", group = "TeleOp")
public class LUTSetup extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DriverDanny driver;
    private ShooterMcGavin shooter;
    private double currentTargetVelocity = 1100;
    private double currentHoodPosition = 0;

    @Override
    public void init() {
        DriverDanny.Alliance startingAlliance = DriverDanny.Alliance.BLUE;
        Pose startingPose = DriverDanny.Poses.BLUE_FAR_START_POSE;

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
        shooter.update();

        double joyY = -gamepad2.left_stick_y; // leaving this inverted so it works for robotCentricDrive (and we adjust for it on fieldCentricDrive)
        double joyX = gamepad2.left_stick_x;
        double rotate = gamepad2.right_stick_x;

        if (gamepad2.aWasPressed() && !shooter.isShooting()) {
            shooter.startShootingAtVelocityAndHoodPosition(currentTargetVelocity, currentHoodPosition);
        }

        if (gamepad2.right_bumper) {
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

        if (gamepad2.dpadUpWasPressed()) {
            currentHoodPosition += 0.05;
        }

        if (gamepad2.dpadDownWasPressed()) {
            currentHoodPosition -= 0.05;
        }

        if (gamepad2.dpadLeftWasPressed()) {
            currentTargetVelocity -= 20;
        }

        if (gamepad2.dpadRightWasPressed()) {
            currentTargetVelocity += 20;
        }

        driver.drive(joyY, joyX, rotate);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("CurrentTargetVelocity", currentTargetVelocity);
        telemetry.addData("CurrentHoodPosition", currentHoodPosition);
        telemetry.update();
    }
}
