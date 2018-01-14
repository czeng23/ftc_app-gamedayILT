package io.github.codecube.cnntest;

import android.app.AlertDialog;
import android.content.DialogInterface;
import android.os.Bundle;
import android.os.Environment;
import android.view.View;
import android.view.WindowManager;
import android.widget.ArrayAdapter;
import android.widget.Spinner;

import org.lasarobotics.vision.opmode.VisionEnabledActivity;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class CNNActivity extends VisionEnabledActivity {
    private List<String> mAvailableModels = new ArrayList<>();
    private CNNTestOpMode mOpMode;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_cnn);

        File caffeFiles = new File(Environment.getExternalStorageDirectory().toString() +
                "/caffe_files");
        if(!caffeFiles.exists()) {
            caffeFiles.mkdir();
        }
        for (File f : caffeFiles.listFiles()) {
            if (f.getName().contains("caffemodel")) {
                mAvailableModels.add(f.getName());
            }
        }
        ArrayAdapter<String> modelsAdapter = new ArrayAdapter<>(this,
                android.R.layout.simple_spinner_item, mAvailableModels);
        modelsAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        ((Spinner) findViewById(R.id.model_selector)).setAdapter(modelsAdapter);

        mOpMode = new CNNTestOpMode();
        initializeVision(R.id.surfaceView, mOpMode);
    }

    @Override
    public void onPause() {
        super.onPause();
        mOpMode.stop();
    }

    public void toggleClassification(View view) {
        int index = ((Spinner) findViewById(R.id.model_selector)).getSelectedItemPosition();
        //No performance impact if the selected network is already loaded
        String name = mAvailableModels.get(index);
        int end = name.length() - 11; //Length of .caffemodel
        mOpMode.loadModel(name.substring(0, end));
        mOpMode.toggleClassification();
    }

    private void captureImages() {
        int label = ((Spinner) findViewById(R.id.label)).getSelectedItemPosition();
        mOpMode.captureImages(label);
    }

    public void captureImages(View view) {
        if(mOpMode.shouldPromtUserForLabel()) {
            new AlertDialog.Builder(this).setTitle("Check")
                    .setMessage("Did you set the label?")
                    .setPositiveButton(android.R.string.yes, new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialogInterface, int i) {
                            captureImages();
                        }
                    })
                    .setNegativeButton(android.R.string.no, null).show();
        } else {
            captureImages();
        }
    }
}
