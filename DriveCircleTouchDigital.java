// simple autonomous program that drives bot in a circle then ends.
// this code assumes it will end before the period is over but if the period ended while
// still driving, this code will just stop. Stops after 5 seconds or on touch sensor button.
// Uses REV Digital Touch sensor.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Autonomous(name="Drive Circle Touch Digital", group="Exercises")
//@Disabled
public class DriveCircleTouchDigital extends LinearOpMode
{
    DcMotor         leftMotor;
    DcMotor         rightMotor;
    DigitalChannel  touch;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        // get a reference to our digitalTouch object.
        touch = hardwareMap.get(DigitalChannel.class, "touch_sensor");

        // set the digital channel to input.
        touch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // set power levels 75% left and 10% right to drive in an arc to the right.

        leftMotor.setPower(-0.75);
        rightMotor.setPower(-0.20);

        resetStartTime();

        // drive until touch sensor button pressed or 5 seconds passes.

        // Note that with digital sensor, not touched returns True, touched returns false.

        while (getRuntime() < 5 && touch.getState()) {idle();}

        // turn the motors off.

        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
}
