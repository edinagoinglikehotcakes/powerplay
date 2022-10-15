package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class TeleOpD extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor hexMotor = hardwareMap.dcMotor.get("hexMotor");
        TouchSensor touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        waitForStart();

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y;
            hexMotor.setPower(power);
            telemetry.addData("Target Power", power);
            telemetry.addData("Actual Power", hexMotor.getPower());
            telemetry.addData("Pressed", touchSensor.isPressed());
            telemetry.update();
        }
    }
}