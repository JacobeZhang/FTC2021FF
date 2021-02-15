package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareBIGBRAINBOTS;


@TeleOp(name = "BBBTeleOpTest")
public class BBBTeleOpTest extends LinearOpMode {
    HardwareBIGBRAINBOTS robot   = new HardwareBIGBRAINBOTS();   // Use BIGBRAINBOTS's hardware

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();
        boolean antiBug = false;
        boolean doneAntiBug = true;
        while (opModeIsActive()) {
            double joystick = gamepad1.right_stick_y;

            robot.RearRightDrive.setPower(joystick);
        }
    }
}
