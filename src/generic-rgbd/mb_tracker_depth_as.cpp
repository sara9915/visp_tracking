
#include <iostream>
#include <visp3/core/vpConfig.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

// Conversion
#include "conversion_camera.h"
#include "conversion_pose.h"
#include "conversion_image.h"

/* ROS */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <visp_tracking/tracking_mode_Action.h>
#include <actionlib/server/simple_action_server.h>
#include <librealsense2/rs.hpp>
#include <pcl_ros/point_cloud.h>

// VISP
#include "visp3/core/vpDisplay.h"
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/vision/vpKeyPoint.h>
#include <opencv2/core/core.hpp>

bool use_ogre = false;
bool use_scanline = false;
bool use_edges = false;
bool use_klt = false;
bool use_depth = true;
bool learn = false;
bool auto_init = false;
bool display_projection_error = true;
double proj_error_threshold = 25;
bool user_init = true;

void conv_pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<pcl::PointXYZ>::Ptr I_element)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *I_element);
}

void conv_rgba_callback(const sensor_msgs::Image::ConstPtr &msg, vpImage<vpRGBa> *I_element)
{
  *I_element = visp_bridge::toVispImageRGBa(*msg);
}

void conv_depth_callback(const sensor_msgs::Image::ConstPtr &msg, vpImage<uint16_t> *I_element)
{
  memcpy(I_element->bitmap, &(msg->data[0]), I_element->getHeight() * I_element->getWidth() * sizeof(uint16_t));
}

bool executeCB(const visp_tracking::tracking_mode_GoalConstPtr &goal, actionlib::SimpleActionServer<visp_tracking::tracking_mode_Action> *as, ros::NodeHandle *nh, std::string *config_color, std::string *config_depth, std::string *model_color, std::string *model_depth, std::string *init_file, std::string *learning_data)
{
  visp_tracking::tracking_mode_Feedback feedback_;
  visp_tracking::tracking_mode_Result result_;

  bool success = true;
  *model_color = goal->path_wrl;
  model_depth = model_color;

  use_depth = goal->use_depth;
  use_edges = goal->use_edges;
  use_klt = goal->use_ktl;

  std::cout << "Use edges: " << goal->use_edges;

  /* Initial configuration */
  std::string parentname = vpIoTools::getParent(*model_color);
  if (config_color->empty())
  {
    *config_color = (parentname.empty() ? "" : (parentname + "/")) + vpIoTools::getNameWE(*model_color) + ".xml";
  }
  if (config_depth->empty())
  {
    *config_depth = (parentname.empty() ? "" : (parentname + "/")) + vpIoTools::getNameWE(*model_color) + "_depth.xml";
  }
  if (init_file->empty())
  {
    *init_file = (parentname.empty() ? "" : (parentname + "/")) + vpIoTools::getNameWE(*model_color) + ".init";
  }
  std::cout << "Tracked features: " << std::endl;
  std::cout << "  Use edges   : " << use_edges << std::endl;
  std::cout << "  Use klt     : " << use_klt << std::endl;
  std::cout << "  Use depth   : " << use_depth << std::endl;
  std::cout << "Tracker options: " << std::endl;
  std::cout << "  Use ogre    : " << use_ogre << std::endl;
  std::cout << "  Use scanline: " << use_scanline << std::endl;
  std::cout << "  Proj. error : " << proj_error_threshold << std::endl;
  std::cout << "  Display proj. error: " << display_projection_error << std::endl;
  std::cout << "Config files: " << std::endl;
  std::cout << "  Config color: "
            << "\"" << *config_color << "\"" << std::endl;
  std::cout << "  Config depth: "
            << "\"" << *config_depth << "\"" << std::endl;
  std::cout << "  Model color : "
            << "\"" << *model_color << "\"" << std::endl;
  std::cout << "  Model depth : "
            << "\"" << *model_depth << "\"" << std::endl;
  std::cout << "  Init file   : "
            << "\"" << *init_file << "\"" << std::endl;
  std::cout << "Learning options   : " << std::endl;
  std::cout << "  Learn       : " << learn << std::endl;
  std::cout << "  Auto init   : " << auto_init << std::endl;
  std::cout << "  Learning data: " << *learning_data << std::endl;

  if (!use_edges && !use_klt && !use_depth)
  {
    std::cout << "You must choose at least one visual features between edge, KLT and depth." << std::endl;
    success = false;
    return EXIT_FAILURE;
  }

  if (config_color->empty() || config_depth->empty() || model_color->empty() || model_depth->empty() || init_file->empty())
  {
    std::cout << "config_color.empty() || config_depth.empty() || model_color.empty() || model_depth.empty() || "
                 "init_file.empty()"
              << std::endl;
    success = false;
    return EXIT_FAILURE;
  }

  auto cam_color_ros = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/color/camera_info");
  vpCameraParameters cam_color = visp_bridge::toVispCameraParameters(*cam_color_ros);

  auto cam_depth_ros = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/depth/camera_info");
  vpCameraParameters cam_depth = visp_bridge::toVispCameraParameters(*cam_depth_ros);

  std::cout << "Sensor internal camera parameters for color camera: " << cam_color << std::endl;
  std::cout << "Sensor internal camera parameters for depth camera: " << cam_depth << std::endl;

  int width_color = cam_color_ros->width;
  int height_color = cam_color_ros->height;
  int width_depth = cam_depth_ros->width;
  int height_depth = cam_depth_ros->height;
  int fps = 30;

  vpImage<vpRGBa> I_color(height_color, width_color);
  vpImage<unsigned char> I_gray(height_color, width_color);
  vpImage<unsigned char> I_depth(height_depth, width_depth);
  vpImage<uint16_t> I_depth_raw(height_depth, width_depth);

  unsigned int _posx = 100, _posy = 50;

#ifdef VISP_HAVE_X11
  vpDisplayX d1, d2;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d1, d2;
#elif defined(VISP_HAVE_OPENCV)
  vpDisplayOpenCV d1, d2;
#endif
  if (use_edges || use_klt)
    d1.init(I_gray, _posx, _posy, "Color stream");
  if (use_depth)
    d2.init(I_depth, _posx + I_gray.getWidth() + 10, _posy, "Depth stream");

  ros::Subscriber color_sub = nh->subscribe<sensor_msgs::Image>("/camera/color/image_raw2", 1, boost::bind(&conv_rgba_callback, _1, &I_color));
  ros::Subscriber depth_sub = nh->subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1, boost::bind(&conv_depth_callback, _1, &I_depth_raw));
  ros::Publisher pose_pub = nh->advertise<geometry_msgs::PoseStamped>("tracker_pose", 1);
  ros::Rate loop_rate(1);
  int i = 0;
  while(i < 10)
  {
    ros::spinOnce();
    i++;
  }
  while (true)
  {
    ros::spinOnce();
    loop_rate.sleep();
    if (use_edges || use_klt)
    {
      vpImageConvert::convert(I_color, I_gray);
      vpDisplay::display(I_gray);
      vpDisplay::displayText(I_gray, 20, 20, "Click when ready.", vpColor::red);
      vpDisplay::flush(I_gray);

      if (vpDisplay::getClick(I_gray, false))
      {
        break;
      }
    }
    if (use_depth)
    {
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

      vpDisplay::display(I_depth);
      vpDisplay::displayText(I_depth, 20, 20, "Click when ready.", vpColor::red);
      vpDisplay::flush(I_depth);

      if (vpDisplay::getClick(I_depth, false))
      {
        break;
      }
    }
  }

  std::vector<int> trackerTypes;
  if (use_edges && use_klt)
    trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
  else if (use_edges)
    trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER);
  else if (use_klt)
    trackerTypes.push_back(vpMbGenericTracker::KLT_TRACKER);

  if (use_depth)
    trackerTypes.push_back(vpMbGenericTracker::DEPTH_DENSE_TRACKER);

  tf::TransformListener listener;
  tf::StampedTransform transform;

  try
  {
    listener.waitForTransform("/camera_color_frame", "/camera_depth_frame", ros::Time(0), ros::Duration(3.0));
    listener.lookupTransform("/camera_color_frame", "/camera_depth_frame",
                             ros::Time(0), transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  geometry_msgs::Transform transform_color_depth;
  transform_color_depth.translation.x = transform.getOrigin().getX();
  transform_color_depth.translation.y = transform.getOrigin().getY();
  transform_color_depth.translation.z = transform.getOrigin().getZ();

  transform_color_depth.rotation.w = transform.getRotation().getW();
  transform_color_depth.rotation.x = transform.getRotation().getX();
  transform_color_depth.rotation.y = transform.getRotation().getY();
  transform_color_depth.rotation.z = transform.getRotation().getZ();

  vpHomogeneousMatrix depth_M_color = visp_bridge::toVispHomogeneousMatrix(transform_color_depth);

  // vpHomogeneousMatrix depth_M_color = realsense.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH);
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraTransformations;
  std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
  std::map<std::string, const vpImage<vpRGBa> *> mapOfImages_mod;
  std::map<std::string, std::string> mapOfInitFiles;
  std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::ConstPtr> mapOfPointclouds;
  std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraPoses;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  ros::Subscriber pointcloud_sub = nh->subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, boost::bind(&conv_pointcloud_callback, _1, pointcloud));

  vpMbGenericTracker tracker(trackerTypes);

  if ((use_edges || use_klt) && use_depth)
  {
    tracker.loadConfigFile(*config_color, *config_depth);
    tracker.loadModel(*model_color, *model_depth);
    std::cout << "Sensor internal depth_M_color: \n"
              << depth_M_color << std::endl;
    mapOfCameraTransformations["Camera2"] = depth_M_color;
    tracker.setCameraTransformationMatrix(mapOfCameraTransformations);
    mapOfImages["Camera1"] = &I_gray;
    mapOfImages["Camera2"] = &I_depth;
    mapOfInitFiles["Camera1"] = *init_file;
    tracker.setCameraParameters(cam_color, cam_depth);
  }
  else if (use_edges || use_klt)
  {
    tracker.loadConfigFile(*config_color);
    tracker.loadModel(*model_color);
    tracker.setCameraParameters(cam_color);
  }
  else if (use_depth)
  {
    tracker.loadConfigFile(*config_depth);
    tracker.loadModel(*model_depth);
    tracker.setCameraParameters(cam_depth);
  }

  tracker.setDisplayFeatures(true);
  tracker.setOgreVisibilityTest(use_ogre);
  tracker.setScanLineVisibilityTest(use_scanline);
  tracker.setProjectionErrorComputation(true);
  tracker.setProjectionErrorDisplay(display_projection_error);

#if (defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D)) || \
    (VISP_HAVE_OPENCV_VERSION >= 0x030411 && CV_MAJOR_VERSION < 4) || (VISP_HAVE_OPENCV_VERSION >= 0x040400)
  std::string detectorName = "SIFT";
  std::string extractorName = "SIFT";
  std::string matcherName = "BruteForce";
#else
  std::string detectorName = "FAST";
  std::string extractorName = "ORB";
  std::string matcherName = "BruteForce-Hamming";
#endif
  vpKeyPoint keypoint;
  if (learn || auto_init)
  {
    keypoint.setDetector(detectorName);
    keypoint.setExtractor(extractorName);
    keypoint.setMatcher(matcherName);
#if !(defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D))
#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
    keypoint.setDetectorParameter("ORB", "nLevels", 1);
#else
    cv::Ptr<cv::ORB> orb_detector = keypoint.getDetector("ORB").dynamicCast<cv::ORB>();
    if (orb_detector)
    {
      orb_detector->setNLevels(1);
    }
#endif
#endif
  }

  if (auto_init)
  {
    if (!vpIoTools::checkFilename(*learning_data))
    {
      std::cout << "Cannot enable auto detection. Learning file \"" << *learning_data << "\" doesn't exist" << std::endl;
      return EXIT_FAILURE;
    }
    keypoint.loadLearningData(*learning_data, true);
  }
  if (!user_init)
  {
    std::cout << "Init from pose";
    geometry_msgs::Pose initial_pose_cm;
    initial_pose_cm.position.x = goal->initial_pose.pose.position.x * 100;
    initial_pose_cm.position.y = goal->initial_pose.pose.position.y * 100;
    initial_pose_cm.position.z = goal->initial_pose.pose.position.z * 100;
    initial_pose_cm.orientation = goal->initial_pose.pose.orientation;
    auto init_pose_ = visp_bridge::toVispHomogeneousMatrix(initial_pose_cm);
    // tracker.initFromPose(I_gray, init_pose_);

    if ((use_edges || use_klt) && use_depth)
    {
      mapOfCameraPoses["Camera1"] = init_pose_;
      mapOfCameraPoses["Camera2"] = depth_M_color * init_pose_;
      tracker.initFromPose(mapOfImages, mapOfCameraPoses);
    }
    else if (use_edges || use_klt)
    {
      tracker.initFromPose(I_gray, init_pose_);
    }
    else if (use_depth)
    {
      tracker.initFromPose(I_depth, depth_M_color * init_pose_);
    }
  }
  else
  {

    if ((use_edges || use_klt) && use_depth)
      tracker.initClick(mapOfImages, mapOfInitFiles, true);
    else if (use_edges || use_klt)
      tracker.initClick(I_gray, *init_file, true);
    else if (use_depth)
      tracker.initClick(I_depth, *init_file, true);

    if (learn)
      vpIoTools::makeDirectory(vpIoTools::getParent(*learning_data));
  }

  bool run_auto_init = false;
  if (auto_init)
  {
    run_auto_init = true;
  }
  std::vector<double> times_vec;

  try
  {
    // To be able to display keypoints matching with test-detection-rs2
    int learn_id = 1;
    bool quit = false;
    bool learn_position = false;
    double loop_t = 0;
    vpHomogeneousMatrix cMo;

    while (!quit)
    {
      // check that preempt has not been requested by the client
      if (as->isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("Preempted tracking as");
        // set the action state to preempted
        as->setPreempted();
        success = false;
      }

      double t = vpTime::measureTimeMs();
      bool tracking_failed = false;

      // Acquire images and update tracker input data
      ros::spinOnce();
      // realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointcloud, NULL, NULL);

      if (use_edges || use_klt || run_auto_init)
      {
        vpImageConvert::convert(I_color, I_gray);
        vpDisplay::display(I_gray);
      }
      if (use_depth)
      {
        vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
        vpDisplay::display(I_depth);
      }

      if ((use_edges || use_klt) && use_depth)
      {
        mapOfImages["Camera1"] = &I_gray;
        mapOfPointclouds["Camera2"] = pointcloud;
        mapOfWidths["Camera2"] = width_depth;
        mapOfHeights["Camera2"] = height_depth;
      }
      else if (use_edges || use_klt)
      {
        mapOfImages["Camera"] = &I_gray;
      }
      else if (use_depth)
      {
        mapOfPointclouds["Camera"] = pointcloud;
        mapOfWidths["Camera"] = width_depth;
        mapOfHeights["Camera"] = height_depth;
      }

      // Run auto initialization from learned data
      if (run_auto_init)
      {
        if (keypoint.matchPoint(I_gray, cam_color, cMo))
        {
          std::cout << "Auto init succeed" << std::endl;
          if ((use_edges || use_klt) && use_depth)
          {
            mapOfCameraPoses["Camera1"] = cMo;
            mapOfCameraPoses["Camera2"] = depth_M_color * cMo;
            tracker.initFromPose(mapOfImages, mapOfCameraPoses);
          }
          else if (use_edges || use_klt)
          {
            tracker.initFromPose(I_gray, cMo);
          }
          else if (use_depth)
          {
            tracker.initFromPose(I_depth, depth_M_color * cMo);
          }
        }
        else
        {
          if (use_edges || use_klt)
          {
            vpDisplay::flush(I_gray);
          }
          if (use_depth)
          {
            vpDisplay::flush(I_depth);
          }
          continue;
        }
      }

      // Run the tracker
      try
      {
        if (run_auto_init)
        {
          // Turn display features off just after auto init to not display wrong moving-edge if the tracker fails
          tracker.setDisplayFeatures(false);

          run_auto_init = false;
        }
        if ((use_edges || use_klt) && use_depth)
        {
          tracker.track(mapOfImages, mapOfPointclouds);
        }
        else if (use_edges || use_klt)
        {
          tracker.track(I_gray);
        }
        else if (use_depth)
        {
          tracker.track(mapOfImages, mapOfPointclouds);
        }
      }
      catch (const vpException &e)
      {
        std::cout << "Tracker exception: " << e.getStringMessage() << std::endl;
        tracking_failed = true;
        if (auto_init)
        {
          std::cout << "Tracker needs to restart (tracking exception)" << std::endl;
          run_auto_init = true;
        }
      }

      // Get object pose
      cMo = tracker.getPose();
      geometry_msgs::PoseStamped current_pose;
      current_pose.pose = visp_bridge::toGeometryMsgsPose(cMo);
      current_pose.pose.position.x = current_pose.pose.position.x; //* 0.01;
      current_pose.pose.position.y = current_pose.pose.position.y; //* 0.01;
      current_pose.pose.position.z = current_pose.pose.position.z; //* 0.01;
      current_pose.header.frame_id = "camera_color_optical_frame";
      current_pose.header.stamp = ros::Time::now();

      // Publish current pose
      pose_pub.publish(current_pose);
      feedback_.pose_tracker = current_pose;
      
      as->publishFeedback(feedback_);

      // Check tracking errors
      double proj_error = 0;
      if (tracker.getTrackerType() & vpMbGenericTracker::EDGE_TRACKER)
      {
        // Check tracking errors
        proj_error = tracker.getProjectionError();
      }
      else
      {
        proj_error = tracker.computeCurrentProjectionError(I_gray, cMo, cam_color);
      }

      if (auto_init && proj_error > proj_error_threshold)
      {
        std::cout << "Tracker needs to restart (projection error detected: " << proj_error << ")" << std::endl;
        run_auto_init = true;
        tracking_failed = true;
      }

      // Display tracking results
      if (!tracking_failed)
      {
        // Turn display features on
        tracker.setDisplayFeatures(true);

        if ((use_edges || use_klt) && use_depth)
        {
          tracker.display(I_gray, I_depth, cMo, depth_M_color * cMo, cam_color, cam_depth, vpColor::red, 3);
          vpDisplay::displayFrame(I_gray, cMo, cam_color, 0.05, vpColor::none, 3);
          vpDisplay::displayFrame(I_depth, depth_M_color * cMo, cam_depth, 0.05, vpColor::none, 3);
        }
        else if (use_edges || use_klt)
        {
          tracker.display(I_gray, cMo, cam_color, vpColor::red, 3);
          vpDisplay::displayFrame(I_gray, cMo, cam_color, 0.05, vpColor::none, 3);
        }
        else if (use_depth)
        {
          tracker.display(I_depth, cMo, cam_depth, vpColor::red, 3);
          vpDisplay::displayFrame(I_depth, cMo, cam_depth, 0.05, vpColor::none, 3);
        }

        {
          std::stringstream ss;
          ss << "Nb features: " << tracker.getError().size();
          vpDisplay::displayText(I_gray, I_gray.getHeight() - 50, 20, ss.str(), vpColor::red);
        }
        {
          std::stringstream ss;
          ss << "Features: edges " << tracker.getNbFeaturesEdge() << ", klt " << tracker.getNbFeaturesKlt()
             << ", depth " << tracker.getNbFeaturesDepthDense();
          vpDisplay::displayText(I_gray, I_gray.getHeight() - 30, 20, ss.str(), vpColor::red);
        }
      }

      std::stringstream ss;
      ss << "Loop time: " << loop_t << " ms";

      vpMouseButton::vpMouseButtonType button;
      if (use_edges || use_klt)
      {
        vpDisplay::displayText(I_gray, 20, 20, ss.str(), vpColor::red);
        if (learn)
          vpDisplay::displayText(I_gray, 35, 20, "Left click: learn  Right click: quit", vpColor::red);
        else if (auto_init)
          vpDisplay::displayText(I_gray, 35, 20, "Left click: auto_init  Right click: quit", vpColor::red);
        else
          vpDisplay::displayText(I_gray, 35, 20, "Right click: quit", vpColor::red);

        vpDisplay::flush(I_gray);

        if (vpDisplay::getClick(I_gray, button, false))
        {
          if (button == vpMouseButton::button3)
          {
            quit = true;
          }
          else if (button == vpMouseButton::button1 && learn)
          {
            learn_position = true;
          }
          else if (button == vpMouseButton::button1 && auto_init && !learn)
          {
            run_auto_init = true;
          }
        }
      }
      if (use_depth)
      {
        vpDisplay::displayText(I_depth, 20, 20, ss.str(), vpColor::red);
        vpDisplay::displayText(I_depth, 40, 20, "Click to quit", vpColor::red);
        vpDisplay::flush(I_depth);

        if (vpDisplay::getClick(I_depth, false))
        {
          quit = true;
        }
      }

      if (learn_position)
      {
        // Detect keypoints on the current image
        std::vector<cv::KeyPoint> trainKeyPoints;
        keypoint.detect(I_gray, trainKeyPoints);

        // Keep only keypoints on the cube
        std::vector<vpPolygon> polygons;
        std::vector<std::vector<vpPoint>> roisPt;
        std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint>>> pair = tracker.getPolygonFaces();
        polygons = pair.first;
        roisPt = pair.second;

        // Compute the 3D coordinates
        std::vector<cv::Point3f> points3f;
        vpKeyPoint::compute3DForPointsInPolygons(cMo, cam_color, trainKeyPoints, polygons, roisPt, points3f);

        // Build the reference keypoints
        keypoint.buildReference(I_gray, trainKeyPoints, points3f, true, learn_id++);

        // Display learned data
        for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it)
        {
          vpDisplay::displayCross(I_gray, (int)it->pt.y, (int)it->pt.x, 10, vpColor::yellow, 3);
        }
        learn_position = false;
        std::cout << "Data learned" << std::endl;
      }
      loop_t = vpTime::measureTimeMs() - t;
      times_vec.push_back(loop_t);
    }
    if (learn)
    {
      std::cout << "Save learning file: " << *learning_data << std::endl;
      keypoint.saveLearningData(*learning_data, true, true);
    }
  }
  catch (const vpException &e)
  {
    std::cout << "Catch an exception: " << e.what() << std::endl;
  }

  if (!times_vec.empty())
  {
    std::cout << "\nProcessing time, Mean: " << vpMath::getMean(times_vec)
              << " ms ; Median: " << vpMath::getMedian(times_vec) << " ; Std: " << vpMath::getStdev(times_vec) << " ms"
              << std::endl;
  }

  if (success)
  {
    ROS_INFO("Tracker AS: Succeeded");
    result_.success = success;
    // set the action state to succeeded
    as->setSucceeded(result_);
  }

  return success;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracker_node");
  ros::NodeHandle nh;

#ifdef VISP_HAVE_PCL
  std::cout << "VISP_HAVE_PCL" << std::endl;
#else
  std::cout << "NOOO VISP_HAVE_PCL" << std::endl;
#endif

  ros::AsyncSpinner spinner(0);
  spinner.start();

  std::string config_color, config_depth;
  std::string model_color, model_depth = "";
  std::string init_file;
  std::string learning_data = "learning/data-learned.bin";

  /* Creazione del ros action */
  actionlib::SimpleActionServer<visp_tracking::tracking_mode_Action> as(nh, "tracker_as", boost::bind(&executeCB, _1, &as, &nh, &config_color, &config_depth, &model_color, &model_depth, &init_file, &learning_data), false);
  as.start();

  ros::waitForShutdown();

  return EXIT_SUCCESS;
}
