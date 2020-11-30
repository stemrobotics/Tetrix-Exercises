package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * This class provides a way to capture video frames (images) from the camera
 * when using Vuforia. Vuforia must own the camera so if we  want to do image
 * processing alongside Vuforia, we have to get the images from it. Once
 * Vuforia is configured create an instance of this class with the localizer
 * object and then you can get images posted by Vuforia for external use.
 * Note that this means the images change on Vuforia's schedule and that can
 * be fairly slow. Even if you call for images rapidly, the image may not
 * change for 10s of milliseconds. This class works with web cams and phones.
 */
class VuforiaCapture
{
    private VuforiaLocalizer    vuforia;

    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;

    /**
     * Constructor.
     * @param vuforia Localizer object created by calling program.
     */
    public VuforiaCapture(VuforiaLocalizer vuforia)
    {
        this.vuforia = vuforia;
    }

    /**
     * Start capturing images from Vuforia. Call after your call
     * to activate on the localizer object.
     */
    public void startCapture()
    {
        vuforia.enableConvertFrameToBitmap();

        vuforia.setFrameQueueCapacity(10);

        frameQueue = vuforia.getFrameQueue();
    }

    /**
     * Get a camera frame from Vuforia if one is available. May take up
     * to 1 ms to return. Vuforia makes the frames available on it's own
     * timing and it may be 10s of ms between new images. If an image is
     * ready, this call returns immediately. If no image available, then
     * this call will wait 1ms for a frame or return null. Internally we
     * queue up to 10 frames but if you call this in the main opmode loop
     * the queue will likely have none or 1 or 2 images. Queue only helps
     * if you are taking frames slower than Vuforia is updating the queue.
     * It this usually the other way around.
     * @return Camera frame (image). Returns null if no image available.
     */
    public Bitmap getFrame()
    {
        if (frameQueue == null) return null;

        try
        {
            return vuforia.convertFrameToBitmap(frameQueue.poll(1, TimeUnit.MILLISECONDS));
        } catch (Exception e) { return null; }
    }
}
