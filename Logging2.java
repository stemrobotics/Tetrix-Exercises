try
{
    while (opModeIsActive() && !stopFlag)
    {
        if (getRuntime() > 5)
        {
            Logging.log("timeout");
            stopFlag = true;
        }

        if (touch.isPressed())
        {
            Logging.log("button touched");
            stopFlag = true;
        }

        idle();
    }
}
catch (Exception e)
{
    // write the exception info to the log file.
    e.printStackTrace(Logging.logPrintStream);
}
