// First we identify the package our OpMode belongs to.

package org.firstinspires.ftc.teamcode;

// Now we import the classes we need from the FTC SDK and the Java SDK.

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

// Here we extend the base OpMode class to be our new MyOpMode class.
// The @Autonomous "annotation" registers our new OpMode with the
// FtcRobotController app so our OpMode appears on the driver station
// menu of OpModes you can run.

@Autonomous(name="MyOpMode")
public class MyOpMode extends LinearOpMode 
{
  private String startDate;
  private ElapsedTime runtime = new ElapsedTime();
  
  // Create a string containing the current date and time.
  
  startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());

  // Pause program, wait for start button press.
  
  waitForStart();
  
  // Start button pressed, off we go.
  
  runtime.reset();
  
  // Now loop displaying the updated run time of this opMode until stop button is pressed.
  
  while (opModeIsActive())
  {
    telemetry.addData("Start", "MyOpMode started at " + startDate);
    telemetry.addData("Status", "running for " + runtime.toString());
    idle();
  }
}
