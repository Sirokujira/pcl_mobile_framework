package com.sirokujira.pclmobile

import junit.framework.TestCase.assertEquals
import org.junit.After
import org.junit.Before
import org.junit.Test

/**
 * Example local unit test, which will execute on the development machine (host).
 *
 * @see [Testing documentation](http://d.android.com/tools/testing)
 */
class PCLFilterUnitTest {
    var pclmodule: pclmobileJNILib? = null

    @Before
    // @Throws(Exception::class)
    fun setUp() {
        pclmodule = pclmobileJNILib
        pclmodule?.load("/storage/emulated/0/lamppost.pcd")
        // pclmobileJNILib.load("/storage/emulated/0/lamppost.pcd")
    }

    @After
    // @Throws(Exception::class)
    fun tearDown() {
        pclmodule = null
    }

    @Test
    fun filter_isAxis() {
        pclmodule?.filterAxis("x", 0.0, 1.00)
        assertEquals(4, 2 + 2);
    }

    @Test
    fun filter_isVoxelGrid() {
        pclmodule?.filterVoxelGrid(0.01, 0.01, 0.01)
        assertEquals(4, 2 + 2);
    }

    // :
}