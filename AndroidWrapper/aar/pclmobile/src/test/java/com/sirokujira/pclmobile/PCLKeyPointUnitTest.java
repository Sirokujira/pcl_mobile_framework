package com.sirokujira.pclmobile;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * Example local unit test, which will execute on the development machine (host).
 *
 * @see <a href="http://d.android.com/tools/testing">Testing documentation</a>
 */
public class PCLKeyPointUnitTest {
    @Before
    public void setUp() throws Exception {

    }

    @After
    public void tearDown() throws Exception {

    }

    @Test
    public void SIFTKeypointTest()
    {
        // assertEquals(4, 2 + 2);
        /*
        PointCloud<KeypointT> keypoints;

        // Compute the SIFT keypoints
        SIFTKeypoint<PointXYZI, KeypointT> sift_detector;
        search::KdTree<PointXYZI>::Ptr tree (new search::KdTree<PointXYZI>);
        sift_detector.setSearchMethod (tree);
        sift_detector.setScales (0.02f, 5, 3);
        sift_detector.setMinimumContrast (0.03f);

        sift_detector.setInputCloud (cloud_xyzi);
        sift_detector.compute (keypoints);

        ASSERT_EQ (keypoints.width, keypoints.points.size ());
        ASSERT_EQ (keypoints.height, 1);
        EXPECT_EQ (keypoints.points.size (), static_cast<size_t> (169));
        EXPECT_EQ (keypoints.header, cloud_xyzi->header);
        EXPECT_EQ (keypoints.sensor_origin_ (0), cloud_xyzi->sensor_origin_ (0));
        EXPECT_EQ (keypoints.sensor_origin_ (1), cloud_xyzi->sensor_origin_ (1));
        EXPECT_EQ (keypoints.sensor_origin_ (2), cloud_xyzi->sensor_origin_ (2));
        EXPECT_EQ (keypoints.sensor_origin_ (3), cloud_xyzi->sensor_origin_ (3));
        EXPECT_EQ (keypoints.sensor_orientation_.w (), cloud_xyzi->sensor_orientation_.w ());
        EXPECT_EQ (keypoints.sensor_orientation_.x (), cloud_xyzi->sensor_orientation_.x ());
        EXPECT_EQ (keypoints.sensor_orientation_.y (), cloud_xyzi->sensor_orientation_.y ());
        EXPECT_EQ (keypoints.sensor_orientation_.z (), cloud_xyzi->sensor_orientation_.z ());

        // Change the values and re-compute
        sift_detector.setScales (0.05f, 5, 3);
        sift_detector.setMinimumContrast (0.06f);
        sift_detector.compute (keypoints);

        ASSERT_EQ (keypoints.width, keypoints.points.size ());
        ASSERT_EQ (keypoints.height, 1);

        // Compare to previously validated output
        const size_t correct_nr_keypoints = 5;
        const float correct_keypoints[correct_nr_keypoints][4] =
        {
            // { x,  y,  z,  scale }
            {-0.9425f, -0.6381f,  1.6445f,  0.0794f},
            {-0.5083f, -0.5587f,  1.8519f,  0.0500f},
            { 1.0265f,  0.0500f,  1.7154f,  0.1000f},
            { 0.3005f, -0.3007f,  1.9526f,  0.2000f},
            {-0.1002f, -0.1002f,  1.9933f,  0.3175f}
        };

        ASSERT_EQ (keypoints.points.size (), correct_nr_keypoints);
        for (size_t i = 0; i < correct_nr_keypoints; ++i)
        {
            EXPECT_NEAR (keypoints.points[i].x, correct_keypoints[i][0], 1e-4);
            EXPECT_NEAR (keypoints.points[i].y, correct_keypoints[i][1], 1e-4);
            EXPECT_NEAR (keypoints.points[i].z, correct_keypoints[i][2], 1e-4);
            EXPECT_NEAR (keypoints.points[i].scale, correct_keypoints[i][3], 1e-4);
        }
        */
    }
}