package io.github.codecube.caffewrapper;

import android.os.Environment;

import com.sh1r0.caffe_android_lib.CaffeMobile;

import org.opencv.core.Mat;

import java.io.File;

import static android.R.attr.name;

/**
 * Created by josh on 10/13/17.
 */

public class CaffeCNN {
    private static final File sdcard = Environment.getExternalStorageDirectory();
    private static final String MODELS_FOLDER = sdcard.getAbsolutePath() + "/caffe_files";
    private String mName = "";
    private CaffeMobile mInstance;
    private int mInputSize = 256;

    //This is here to load the native caffe libraries.
    static {
        System.loadLibrary("caffe");
        System.loadLibrary("caffe_jni");
    }

    /**
     * Creates a blank instance with no network loaded. Also, the inputSize is set to the default
     * value of 256.
     */
    public CaffeCNN() {
        mInstance = new CaffeMobile();
        mInstance.setNumThreads(1);
    }

    /**
     * Creates a new instance that will accept images of size inputSizexinputSizex3.
     * @param inputSize The width and height of images that will be inputted.
     */
    public CaffeCNN(int inputSize) {
        mInstance = new CaffeMobile();
        mInstance.setNumThreads(1);
        mInputSize = inputSize;
    }

    /**
     * Crates a new instance and loads a model with the given name using loadModel(). inputSize will
     * be set to its default value of 256.
     * @param modelName The name of the model.
     */
    public CaffeCNN(String modelName) {
        mInstance = new CaffeMobile();
        mInstance.setNumThreads(1);
        loadModel(modelName);
    }

    /**
     * Creates a new instance which accepts images of size inputSizexinputSizex3 and also loads a
     * model with the given name using loadModel().
     * @param modelName The name of the model to load.
     * @param inputSize The width and height of images that will be inputted.
     */
    public CaffeCNN(String modelName, int inputSize) {
        mInstance = new CaffeMobile();
        mInstance.setNumThreads(1);
        mInputSize = inputSize;
        loadModel(modelName);
    }

    /**
     * Attempts to load a pretrained model from the phone with the given name. Models should be
     * located in the /sdcard/caffe_files/ directory. For every model, there should be a file
     * [model_name].prototext and [model_name].caffemodel. These are automatically loaded onto
     * the phone if you use the 'Upload To Android Device` feature in model_manager.py.
     * @param modelName The name of the model, used to find the prototext and caffemodel files.
     */
    public void loadModel(String modelName) {
        mName = modelName;
        String path = MODELS_FOLDER + "/" + name;
        //Load the model using the native library.
        mInstance.loadModel(path + ".prototext", path + ".caffemodel");
    }

    /**
     * Sets the size that should be expected for input images.
     * @param inputSize The width and height that all input images should be expected to have.
     */
    public void setInputSize(int inputSize) {
        mInputSize = inputSize;
    }

    /**
     * Runs an image through the currently loaded network.
     * @param image The image to process.
     * @return An array containing the activations of the final layer of neurons in the currently
     * loaded network.
     */
    public float[] classify(Mat image) {
        //Data holds raw WxHxC data from the OpenCV image, transposed holds the CxHxW version used
        //for Caffe.
        byte[] data = new byte[mInputSize * mInputSize * 3],
                transposed = new byte[3 * mInputSize * mInputSize];
        int index = 0;
        image.get(0, 0, data);
        //OpenCV images are ordered by width x height x channel, but Caffe expects them to be in
        //channel x height x width order, which is why this painfulness is necessary:
        for(int x = 0; x < mInputSize; x++) {
            for(int y = 0; y < mInputSize; y++) {
                for(int c = 0; c < 3; c++) {
                    transposed[((c * mInputSize) + y) * mInputSize + x] = data[index];
                    index++;
                }
            }
        }
        //Execute the network using the native library.
        return mInstance.getConfidenceScore(transposed, mInputSize, mInputSize);
    }
}
