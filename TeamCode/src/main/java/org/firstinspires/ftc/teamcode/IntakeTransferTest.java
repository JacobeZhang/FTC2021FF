package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "IntakeTransferTest")
public class IntakeTransferTest extends LinearOpMode {
    private DcMotor IntakeMotor;
    private DcMotor TransferMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        IntakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        TransferMotor = hardwareMap.get(DcMotor.class, "TransferMotor");

        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            IntakeMotor.setPower(1);
            TransferMotor.setPower(1);
        }
    }
}
