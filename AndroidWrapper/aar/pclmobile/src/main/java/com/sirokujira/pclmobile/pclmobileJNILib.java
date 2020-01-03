/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.sirokujira.pclmobile;

/* Wrapper for native library */

public class pclmobileJNILib {

     static {
         System.loadLibrary("native-lib");
     }

     /**
     * @param width the current view width
     * @param height the current view height
     */
     public static native void init(int width, int height);
     public static native void step();
     public static native void load(String filename);

    //region Feature
    public static native void feature1(String filename);
    //endregion

    //region Filter
    /**
     * Filtrering axis min/max area
     * @param filename
     * @param min
     * @param max
     */
    public static native void filterAxis(String filename, double min, double max);

    /**
     * Filtering VoxelGrid
     * @param x
     * @param y
     * @param z
     */
    public static native void filterVoxelGrid(double x, double y, double z);
    //endregion

    // region Geometry
    public static native void geometry1(String filename);
    // endregion

    // region KdTree

    /**
     *
     * @param filename
     */
    public static native void kdtree1(String filename);
    // endregion

    // region KeyPoint

    /**
     * Extraction KeyPoints.
     *
     * @param filename
     */
    public static native void keypoint1(String filename);
    // endregion

    // region Octree

    /**
     *
     * @param filename
     */
    public static native void octree1(String filename);
    // endregion

    // region People

    /**
     *
     * @param filename
     */
    public static native void people1(String filename);
    // endregion

    // region RangeImages
    /**
     *
     * @param filename
     */
    public static native void rangeimages1(String filename);
    // endregion

    // region Recognition
    /**
     *
     * @param filename
     */
    public static native void recognition1(String filename);
    // endregion

    // region Registration

    /**
     *
     * @param filename
     */
    public static native void registration1(String filename);
    // endregion

    // region SampleConsensus

    /**
     *
     * @param filename
     */
    public static native void sampleconsensus1(String filename);
    // endregion

    // region Segmentation

    /**
     *
     * @param filename
     */
    public static native void segmentation1(String filename);
    // endregion

    // region Stereo

    /**
     *
     * @param filename
     */
    public static native void stereo1(String filename);
    // endregion

    // region Surface

    /**
     *
     * @param filename
     */
    public static native void surface1(String filename);
    // endregion

    // region Tracking
    /**
     *
     * @param filename
     */
    public static native void tracking1(String filename);
    // endregion
}
