package com.pcl_mobile;

import android.app.Activity;
import android.os.Bundle;
import com.sirokujira.pclmobile.pclmobileJNILib;
public class MainActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        pclmobileJNILib.load("/storage/emulated/0/lamppost.pcd");
        // Example of a call to a native method
        android.util.Log.d("MainActivity", "test");
    }

}
