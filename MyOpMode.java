// First we identify the package our OpMode belongs to.

package org.firstinspires.ftc.teamcode;

// Now we import the classes we need from the FTC SDK and the Java SDK.

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;

// Here we extend the base OpMode class to be our new MyOpMode class.
// The @Autonomous "annotation" registers our new OpMode with the
// FtcRobotController app so our OpMode appears on the driver station
// menu of OpModes you can run.

@Autonomous(name="MyOpMode")
public class MyOpMode extends OpMode 
{
  private String startDate;
  private ElapsedTime runtime = new ElapsedTime();

  // Here is the init() method. We don't have anything to do here so we could
  // have left it out.

  @Override
  public void init() 
  {
  }

  // Here we are intializing the variables we are using each time we run the
  // OpMode.

  @Override
  public void init_loop() 
  {
    startDate = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss").format(new Date());
    runtime.reset();
    telemetry.addData("MyOpMode Init Loop", runtime.toString());
  }

  // Here are the start() and stop() methods. We don't have anything to do at those
  // times so we could have left them out.

  @Override
  public void start()
  {
  }

  @Override
  public void stop()
  {
  }

  // The loop() method is called over and over until the Stop button is pressed.
  // The method displays the elapsed run time on the driver stattion using the
  // telemtry field of the base OpMode class.

  @Override
  public void loop() 
  {
    telemetry.addData("1 Start", "MyOpMode started at " + startDate);
    telemetry.addData("2 Status", "running for " + runtime.toString());
  }
}
