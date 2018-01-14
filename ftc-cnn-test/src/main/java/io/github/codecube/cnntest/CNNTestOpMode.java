package io.github.codecube.cnntest;

import android.graphics.Bitmap;
import android.os.Environment;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.image.Drawing;
import org.lasarobotics.vision.opmode.TestableVisionOpMode;
import org.lasarobotics.vision.util.color.ColorGRAY;
import org.lasarobotics.vision.util.color.ColorRGBA;
import org.opencv.android.Utils;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.text.DecimalFormat;

import io.github.codecube.caffewrapper.CaffeCNN;

/**
 * Created by josh on 10/9/17.
 */

public class CNNTestOpMode extends TestableVisionOpMode {
    private CaffeCNN mCaffeCnn = new CaffeCNN();
    private String mCurrentModelName = "";
    private boolean mClassify = false;
    private int mImagesToCapture = 0;
    private int mImageLabel = 0;
    private long mLastImageTime = 0;
    private static final int IMAGE_INTERVAL = 200;

    /**
     * Called when the op mode is starting. This is used to set up the camera.
     */
    @Override
    public void init() {
        super.init();

        this.setCamera(Cameras.PRIMARY); //Use the primary camera.
        //640x480 is the smallest standard resolution that is still larger than 256x256. Trying to
        //directly use 256x256 causes it to snap to the nearest standard resolution, 320x240.
        this.setFrameSize(new Size(640, 480));
    }

    /**
     * Tries to load in a different model from the sd card. This uses CaffeCNN.loadModel().
     * @param newModelName The name of the model to load.
     */
    public void loadModel(String newModelName) {
        if(mCurrentModelName != newModelName) {
            mCaffeCnn.loadModel(newModelName);
            mCurrentModelName = newModelName;
        }
    }

    private static final int EXPORT_SIZE = 256; //How big the output images should be.
    private Bitmap mInter = null; //An intermediate bitmap used when scaling images for output.
    private Mat mScaled = null; //An intermediate mat used when scaling images for output.

    /**
     * Called when there is a frame available from the camera. This does many things. Firstly, it
     * captures images (if captureImages() was called earlier and the counter has still not run out)
     * and save them to the SD card with the appropriate labeling to be used during training. Next,
     * it will execute the currently loaded network if classification is currently enabled. It will
     * then draw the results of this on to the frame. Finally, the current status of the image
     * capture counter (if it is not at 0) will be drawn on the frame. The frame will then be
     * returned to be displayed to the user.
     * @param rgba An RGBA Mat representation of the frame the camera captured.
     * @param gray A Mat with no colors representing the frame the camera captured.
     * @return A Mat to display on the screen.
     */
    @Override
    public Mat frame(Mat rgba, Mat gray) {
        if(mImagesToCapture > 0) {
            //Wait until IMAGE_INTERVAL has elapsed to take another picture.
            if(mLastImageTime + IMAGE_INTERVAL < System.currentTimeMillis()) {
                mLastImageTime = System.currentTimeMillis();
                if(mScaled == null) {
                    mScaled = new Mat(); //Initialize this if it has not already been initialized.
                }
                //Resize the input image onto the temporary mat.
                Imgproc.resize(rgba, mScaled, new Size(EXPORT_SIZE, EXPORT_SIZE));
                try {
                    //Try to convert that to a bitmap for exporting.
                    if(mInter == null) {
                        mInter = Bitmap.createBitmap(EXPORT_SIZE, EXPORT_SIZE,
                                Bitmap.Config.ARGB_8888);
                    }
                    Utils.matToBitmap(mScaled, mInter);
                } catch (CvException e) {
                    return rgba;
                }
                //Make sure the export folder exists. If not, create it.
                File folder = new File(Environment.getExternalStorageDirectory() +
                        "/caffe_files/training_images");
                if(!folder.exists()) {
                    folder.mkdir();
                }
                //Make sure to append _LABEL_[label_number] to the end of the file name!
                File output = new File(folder, mLastImageTime + "_LABEL_" + mImageLabel + ".png");
                try {
                    //Try to save the image to a file.
                    mInter.compress(Bitmap.CompressFormat.PNG, 100,
                            new FileOutputStream(output));
                } catch(FileNotFoundException e) { }
                mImagesToCapture--;
            }
            //Show how many images are left to capture.
            Drawing.drawText(rgba, mImagesToCapture + " left to capture.", new Point(50, 24), 1.0f,
                    new ColorGRAY(255));
        }
        if(mClassify) {
            if(mScaled == null) {
                mScaled = new Mat();
            }
            //Resize the camera image to what the network will expect.
            Imgproc.resize(rgba, mScaled, new Size(EXPORT_SIZE, EXPORT_SIZE));

            //Draw neuron activations as percentages with two decimal places.
            float[] classification = mCaffeCnn.classify(mScaled);
            DecimalFormat percent = new DecimalFormat();
            percent.setMaximumFractionDigits(2);
            for(int i = 0; i < 10; i++) {
                Drawing.drawText(rgba, percent.format(classification[i]*100.0f) + "%",
                        new Point(0, 24 * (i + 1)), 1.0f, new ColorRGBA(0, 0, 255, 255));
            }

            return rgba;
        }
        return rgba;
    }

    /**
     * If classification is on, turn it off, and vice versa.
     * @return True if classification is on, after applying the toggle.
     */
    public boolean toggleClassification() {
        mClassify = !mClassify;
        return mClassify;
    }

    private long lastLabelCheck = 0;
    private static final long PROMPT_USER_INTERVAL = 5000;
    public static final int CAPTURE_BATCH_SIZE = 100;

    /**
     * Starts a counter to capture images from the camera as training data. Specifically, it adds
     * CAPTURE_BATCH_SIZE to an internal counter for how many images to capture.
     * @param label What label the captured images should have.
     */
    public void captureImages(int label) {
        mImagesToCapture += CAPTURE_BATCH_SIZE;
        mImageLabel = label;
    }

    //This should be moved, it's more of a GUI thing.
    public boolean shouldPromtUserForLabel() {
        boolean tr = System.currentTimeMillis() > lastLabelCheck + PROMPT_USER_INTERVAL;
        lastLabelCheck = System.currentTimeMillis();
        return tr;
    }
}
