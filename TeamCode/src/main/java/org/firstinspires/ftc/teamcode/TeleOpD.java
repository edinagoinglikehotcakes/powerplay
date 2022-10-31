package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@TeleOp
public class TeleOpD extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor motorLift = hardwareMap.dcMotor.get("motorLift");
        TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        Servo servoClaw = hardwareMap.get(Servo.class, "servoClaw");

        waitForStart();

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            motorLift.setPower(power);
            if (gamepad1.y) {
                servoClaw.setPosition(1);
            }
            if (gamepad1.x) {
                servoClaw.setPosition(0);
            }
            double servoClawPosition = servoClaw.getPosition();
            telemetry.addData("Target Power", power);
            telemetry.addData("Actual Power", motorLift.getPower());
            telemetry.addData("Pressed", limitSwitch.isPressed());
            telemetry.addData("Servo", servoClawPosition);
            telemetry.update();
        }
    }
}