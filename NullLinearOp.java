package org.firstinspires.ftc.teamcode;

// Import the LinearOpMode class as well as the other classes we need.

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

// Our new OpMode class extends the base LinearOpMode class.

@Autonomous(name="NullLinearOp")
public class NullLinearOp extends LinearOpMode
{
  private String startDate, initDate;
  private ElapsedTime runtime = new ElapsedTime();

  // There is only one method for us to override with our own code.

  @Override
  public void runOpMode()  throws InterruptedException
  {
    initDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());

    telemetry.addData("0 init", "NullLinearOp initialized at " + initDate);
    telemetry.update();

    // After we are done initializing our code, we wait for Start button.

    waitForStart();

    // Start button pressed, off we go.

    startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
    runtime.reset();

    // Here we implement the loop our code needs to run in for the duration of
    // our OpModes execution. We can tell when to stop by monitoring the base
    // LinearOpMode class opModeIsActive method.

    while (opModeIsActive())
    {
      telemetry.addData("1 Start", "NullLinearOp started at " + startDate);
      telemetry.addData("2 Status", "running for " + runtime.toString());
      telemetry.update();

      idle();
    }
  }
}
