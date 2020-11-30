package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.concurrent.BlockingQueue;

/**
 * This class provides a way to capture video images from the camera
 * when using Vuforia. This allows you use the images for other purposes
 * like streaming to a browser or processing with OpenCV.
 *
 * Vuforia must own the camera so if we  want to do image
 * processing alongside Vuforia, we have to get the images from Vuforia. Once
 * Vuforia is configured create an instance of this class with the localizer
 * object and then you can get images posted by Vuforia for external use.
 * Note that this means the images change on Vuforia's schedule and that can
 * be fairly slow. Even if you call for images rapidly, the image may not
 * change for 10s of milliseconds. However testing shows adequate an refresh
 * rate for useful video processing. This class uses a separate thread to
 * monitor Vuforia for new images so the calling thread is not blocked
 * waiting for a new image. This class works with web cams and phones.
 */
class VuforiaCapture
{
    private VuforiaLocalizer    vuforia;

    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;

    private Bitmap          currentImage;

    private Object          lock = new Object();

    private MonitorVuforia monitorThread;

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

        vuforia.setFrameQueueCapacity(1);

        frameQueue = vuforia.getFrameQueue();

        monitorThread = new MonitorVuforia();

        monitorThread.start();
    }

    /**
     * Get the current (last posted) image from Vuforia. This image is
     * updated by vuforia on its own schedule typically in the 10s of
     * milliseconds.
     * @return Camera image. Null if no image available.
     */
    public Bitmap getImage()
    {
        synchronized (lock) { return currentImage; }
    }

    // Thread to monitor Vuforia for camera images passed through the blocking queue.

    private class MonitorVuforia extends Thread
    {
        Bitmap  newImage;

        public void run()
        {
            try
            {
                while (true)
                {
                    // Note that take() waits until there is something in the queue to take.
                    // This means this loop will iterate at the speed at which Vuforia updates
                    // the queue. The queue length of 1 assumes this loop will empty the queue
                    // as rapidly as Vuforia updates it. Vuforia returns frames which contain
                    // more than one image format so we extract the bitmap image from the frame.
                    // We copy the retrieved image so that is can be available to callers during
                    // the time this thread waits for a new image.

                    newImage = vuforia.convertFrameToBitmap(frameQueue.take());

                    synchronized (lock)
                    {
                        currentImage = newImage.copy(newImage.getConfig(), false);
                    }
                }
            } catch (Exception e) { e.printStackTrace(); }
        }
    }
}
