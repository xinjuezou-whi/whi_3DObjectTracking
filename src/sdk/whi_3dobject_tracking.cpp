/******************************************************************
3D object tracking to get the 6DOF pose under ROS 1

Features:
- 6DOF pose
- xxx

Dependencies:
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_3dobject_tracking/whi_3dobject_tracking.h"
#include "whi_interfaces/WhiSrvTcpPose.h"

#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <angles/angles.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <m3t/tracker.h>
#include <m3t/renderer_geometry.h>
#include <m3t/realsense_camera.h>
#include <m3t/renderer_geometry.h>
#include <m3t/normal_viewer.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/region_model.h>
#include <m3t/depth_model.h>
#include <m3t/region_modality.h>
#include <m3t/texture_modality.h>
#include <m3t/depth_modality.h>
#include <m3t/link.h>
#include <m3t/static_detector.h>

#include <boost/endian/conversion.hpp>

template <typename T>
static int signOf(T Val)
{
    return T(0) == Val ? 1 : (T(0) < Val) - (Val < T(0));
}

namespace whi_3DObjectTracking
{
    TriDObjectTracking::TriDObjectTracking(std::shared_ptr<ros::NodeHandle>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    TriDObjectTracking::~TriDObjectTracking()
    {
        if (th_tracking_.joinable())
        {
            th_tracking_.join();
        }
    }

    void TriDObjectTracking::init()
    {
        /// init infrastructure
        // pose frame
        node_handle_->param("pose_frame", pose_frame_, std::string("world"));
        geometry_msgs::TransformStamped trans;
        if (retrieveTransform(node_handle_, "world_to_tcp", trans))
        {
            world_to_tcp_ = std::make_shared<geometry_msgs::TransformStamped>(trans);
        }
        if (retrieveTransform(node_handle_, "object_to_tcp", trans))
        {
            object_to_tcp_ = std::make_shared<geometry_msgs::TransformStamped>(trans);
        }
        std::vector<double> positionRef;
        node_handle_->getParam("position_reference", positionRef);
        for (size_t i = 0; i < positionRef.size(); ++i)
        {
            position_reference_[i] = positionRef[i];
        }
        std::vector<double> multipliers;
        node_handle_->getParam("euler_muliplier", multipliers);
        for (size_t i = 0; i < multipliers.size(); ++i)
        {
            euler_multipliers_[i] = multipliers[i];
        }
        
        // publisher
        std::string poseTopic;
        node_handle_->param("pose_topic", poseTopic, std::string());
        if (!poseTopic.empty())
        {
            pub_pose_ = std::make_unique<ros::Publisher>(
                node_handle_->advertise<whi_interfaces::WhiTcpPose>(poseTopic, 1));
        }
        image_transport_ = std::make_unique<image_transport::ImageTransport>(*node_handle_);
        std::string colorTopic;
        node_handle_->param("color_topic", colorTopic, std::string());
        if (!colorTopic.empty())
        {
            pub_color_ = std::make_shared<image_transport::Publisher>(image_transport_->advertise(colorTopic, 1));
        }
        std::string colorOverlayTopic;
        node_handle_->param("color_overlay_topic", colorOverlayTopic, std::string());
        if (!colorOverlayTopic.empty())
        {
            pub_color_overlay_ = std::make_shared<image_transport::Publisher>(
                image_transport_->advertise(colorOverlayTopic, 1));
        }
        std::string depthTopic;
        node_handle_->param("depth_topic", depthTopic, std::string());
        if (!depthTopic.empty())
        {
            pub_depth_ = std::make_shared<image_transport::Publisher>(image_transport_->advertise(depthTopic, 1));
        }
        std::string depthOverlayTopic;
        node_handle_->param("depth_overlay_topic", depthOverlayTopic, std::string());
        if (!depthOverlayTopic.empty())
        {
            pub_depth_overlay_ = std::make_shared<image_transport::Publisher>(
                image_transport_->advertise(depthOverlayTopic, 1));
        }
        // service client
        std::string poseService;
        node_handle_->param("pose_service", poseService, std::string());
        if (!poseService.empty())
        {
            client_pose_ = std::make_unique<ros::ServiceClient>(
                node_handle_->serviceClient<whi_interfaces::WhiSrvTcpPose>(poseService));
        }

        /// init M3T
        initM3t();
    }

    void TriDObjectTracking::initM3t()
    {
        bool viewColor = true;
        bool viewDepth = true;
        bool visualizePoseResult = true;
        bool useRegionModality = true;
        bool useDepthModality = true;
        bool useTextureModality = false;
        bool measureOcclusions = false;
        bool modelOcclusions = false;
        bool align2Color = false;
        node_handle_->param("view_color", viewColor, true);
        node_handle_->param("view_depth", viewDepth, true);
        node_handle_->param("visualize_pose_result", visualizePoseResult, true);
        node_handle_->param("use_region_modality", useRegionModality, true);
        node_handle_->param("use_depth_modality", useDepthModality, true);
        node_handle_->param("use_texture_modality", useTextureModality, false);
        node_handle_->param("measure_occlusions", measureOcclusions, false);
        node_handle_->param("model_occlusions", modelOcclusions, false);
        node_handle_->param("align_to_color", align2Color, false);
        std::vector<std::string> bodyNames;
        node_handle_->getParam("bodies", bodyNames);
        std::vector<int> resolutions;
        node_handle_->getParam("image_resolution", resolutions);
        std::vector<int> frameRates;
        node_handle_->getParam("frame_rates", frameRates);
        bool printIntrinsics;
        node_handle_->getParam("print_intrinsics", printIntrinsics);

        // setup tracker and renderer geometry
        auto tracker{ std::make_shared<m3t::Tracker>("tracker") };
        auto rendererGeometry{ std::make_shared<m3t::RendererGeometry>("renderer geometry") };
        // create cameras
        auto colorCamera{ std::make_shared<m3t::RealSenseColorCamera>("realsense_color",
            resolutions[0], resolutions[1], frameRates[0]) };
        auto depthCamera{ std::make_shared<m3t::RealSenseDepthCamera>("realsense_depth",
            resolutions[0], resolutions[1], frameRates[1], align2Color) };
        // setup viewers
        // color viewer
        auto colorViewer{ std::make_shared<m3t::NormalColorViewer>("color_viewer",
            colorCamera, rendererGeometry) };
        colorViewer->set_display_images(viewColor);
        colorViewer->registerViewImageCallback(std::bind(&TriDObjectTracking::colorImageCallback,
            this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TriDObjectTracking::colorOverlayImageCallback,
            this, std::placeholders::_1, std::placeholders::_2));
        //if (kSaveImages) color_viewer_ptr->StartSavingImages(save_directory, "bmp");
        tracker->AddViewer(colorViewer);
        // depth viewer
        auto depthViewer{ std::make_shared<m3t::NormalDepthViewer>("depth_viewer",
            depthCamera, rendererGeometry, 0.3f, 1.0f)};
        depthViewer->set_display_images(viewDepth);
        depthViewer->registerViewImageCallback(std::bind(&TriDObjectTracking::depthImageCallback,
            this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&TriDObjectTracking::depthOverlayImageCallback,
            this, std::placeholders::_1, std::placeholders::_2));
        //if (kSaveImages) depth_viewer_ptr->StartSavingImages(save_directory, "bmp");
        tracker->AddViewer(depthViewer);
        // setup depth renderer
        auto colorDepthRenderer{ std::make_shared<m3t::FocusedBasicDepthRenderer>("color_depth_renderer",
            rendererGeometry, colorCamera) };
        auto depthDepthRenderer{ std::make_shared<m3t::FocusedBasicDepthRenderer>("depth_depth_renderer",
            rendererGeometry, depthCamera) } ;
        // set up silhouette renderer
        auto colorSilhouetteRenderer{ std::make_shared<m3t::FocusedSilhouetteRenderer>("color_silhouette_renderer",
            rendererGeometry, colorCamera) };
        // setup bodies
        for (const auto& bodyName : bodyNames)
        {
            std::string dirPath;
            node_handle_->param("directory", dirPath, std::string());
            const std::filesystem::path directory{ dirPath };

            // setup body
            std::filesystem::path metafilePath{ directory / (bodyName + ".yaml") };
            auto body{ std::make_shared<m3t::Body>(bodyName, metafilePath) };
            rendererGeometry->AddBody(body);
            colorDepthRenderer->AddReferencedBody(body);
            depthDepthRenderer->AddReferencedBody(body);
            colorSilhouetteRenderer->AddReferencedBody(body);
            // setup models
            auto regionModel{ std::make_shared<m3t::RegionModel>(bodyName + "_region_model", body,
                directory / (bodyName + "_region_model.bin")) };
            auto depthModel{ std::make_shared<m3t::DepthModel>(bodyName + "_depth_model", body,
                directory / (bodyName + "_depth_model.bin")) };
            // setup modalities
            auto regionModality{ std::make_shared<m3t::RegionModality>(bodyName,
                body, colorCamera, regionModel) };
            regionModality->registerPoseResultCallback(std::bind(&TriDObjectTracking::poseCallback,
                this, std::placeholders::_1, std::placeholders::_2));
            auto textureModality{ std::make_shared<m3t::TextureModality>(bodyName + "_texture_modality",
                body, colorCamera, colorSilhouetteRenderer) };
            auto depthModality{ std::make_shared<m3t::DepthModality>(bodyName + "_depth_modality",
                body, depthCamera, depthModel)};
            if (visualizePoseResult)
            {
                regionModality->set_visualize_pose_result(true);
            }
            if (measureOcclusions)
            {
                regionModality->MeasureOcclusions(depthCamera);
                textureModality->MeasureOcclusions(depthCamera);
                depthModality->MeasureOcclusions();
            }
            if (modelOcclusions)
            {
                regionModality->ModelOcclusions(colorDepthRenderer);
                textureModality->ModelOcclusions(colorDepthRenderer);
                depthModality->ModelOcclusions(depthDepthRenderer);
            }
            // setup link
            auto link{ std::make_shared<m3t::Link>(bodyName + "_link", body) };
            if (useRegionModality)
            {
                link->AddModality(regionModality);
            }
            if (useTextureModality)
            {
                link->AddModality(textureModality);
            }
            if (useDepthModality)
            {
                link->AddModality(depthModality);
            }
            // setup optimizer
            auto optimizer{ std::make_shared<m3t::Optimizer>(bodyName + "_optimizer", link) };
            tracker->AddOptimizer(optimizer);
            // setup detector
            std::filesystem::path detectorPath{ directory / (bodyName + "_static_detector.yaml") };
            auto detector{std::make_shared<m3t::StaticDetector>(bodyName, detectorPath, optimizer)};
            tracker->AddDetector(detector);
        }
        // start tracking
        if (tracker->SetUp())
        {
            if (printIntrinsics)
            {
                auto intrinsicsColor = colorCamera->intrinsics();
                std::cout << "color intrinsics:" << std::endl;
                std::cout << "fu: " << intrinsicsColor.fu << " fv: " << intrinsicsColor.fv <<
                    ", pu: " << intrinsicsColor.ppu << " pv: " << intrinsicsColor.ppv << std::endl;
                auto intrinsicsDepth = depthCamera->intrinsics();
                std::cout << "depth intrinsics:" << std::endl;
                std::cout << "fu: " << intrinsicsDepth.fu << " fv: " << intrinsicsDepth.fv <<
                    ", pu: " << intrinsicsDepth.ppu << " pv: " << intrinsicsDepth.ppv << std::endl;
            }

            // acquire the link2world poses
            for (const auto& it : tracker->detector_ptrs())
            {
                Eigen::Isometry3f pose;
                pose.translation() = ((m3t::StaticDetector*)it.get())->link2world_pose().translation();
                pose.linear() = ((m3t::StaticDetector*)it.get())->link2world_pose().rotation();
                link_2_world_pose_map_[it->name()] = pose.cast<double>();
            }

            th_tracking_ = std::thread
            {
                [this, tracker]() -> void
                {
                    bool trackingOnStart = false;
                    this->node_handle_->param("tracking_on_start", trackingOnStart, false);
                    tracker->RunTrackerProcess(true, trackingOnStart);
                }
            };
        }
    }

    void TriDObjectTracking::poseCallback(const std::string& Object, const Eigen::Isometry3d& Pose)
    {
        // align the orientation to link in world frame
        auto alignedMsg = Eigen::toMsg(Pose);
        std::map<std::string, Eigen::Isometry3d>::iterator foundLink;
        if (foundLink = link_2_world_pose_map_.find(Object); foundLink != link_2_world_pose_map_.end())
        {
#ifdef DEBUG
            // the Eigen::eulerAngles method consistently chooses to minimize first angles
            // refer to https://stackoverflow.com/questions/33895970/about-eulerangles-conversion-from-eigen-c-library
            // and https://answers.ros.org/question/323102/getting-eigen3-matrix3feulerangles-values-similar-to-tf2-matrix3x3fgetrpy-roll-pitch-yaw/
            auto eulerL = foundLink->second.linear().eulerAngles(0, 1, 2);
            auto eulerR = foundLink->second.rotation().eulerAngles(0, 1, 2);
            Eigen::Quaterniond q(foundLink->second.rotation());
            auto eulerQ = q.toRotationMatrix().eulerAngles(0, 1, 2);
            std::cout << "Euler from Eigen: " << angles::to_degrees(eulerL[0]) << ","
                << angles::to_degrees(eulerL[1]) << "," << angles::to_degrees(eulerL[2])
                << " : " << angles::to_degrees(eulerR[0]) << "," << angles::to_degrees(eulerR[1])
                << "," << angles::to_degrees(eulerR[2])
                << " : " << angles::to_degrees(eulerQ[0]) << "," << angles::to_degrees(eulerQ[1])
                << "," << angles::to_degrees(eulerQ[2]) << std::endl;
            Eigen::Isometry3d aligned(Pose);
            aligned.linear() = foundLink->second.linear().inverse() * aligned.linear();
#endif
#ifdef DEBUG
            auto link = Eigen::toMsg(foundLink->second);
            tf2::Quaternion qLink(link.orientation.x, link.orientation.y,
                link.orientation.z, link.orientation.w);
            double rollLink = 0.0, pitchLink = 0.0, yawLink = 0.0;
  		    tf2::Matrix3x3(qLink).getRPY(rollLink, pitchLink, yawLink);
            std::cout << "link roll:" << angles::to_degrees(rollLink) << ",pitch:" <<
                angles::to_degrees(pitchLink) << ",yaw:" << angles::to_degrees(yawLink) << std::endl;
#endif
            Eigen::Isometry3d transformLink(foundLink->second);
            auto transformLinkMsg = Eigen::toMsg(transformLink);
            tf2::Quaternion q(transformLinkMsg.orientation.x, transformLinkMsg.orientation.y,
                transformLinkMsg.orientation.z, transformLinkMsg.orientation.w);
            double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            tf2::Quaternion qLink(alignedMsg.orientation.x, alignedMsg.orientation.y,
                alignedMsg.orientation.z, alignedMsg.orientation.w);
            double linkRoll = 0.0, linkPitch = 0.0, linkYaw = 0.0;
  		    tf2::Matrix3x3(qLink).getRPY(linkRoll, linkPitch, linkYaw);
#ifdef DEBUG
            std::cout << "link trans roll:" << angles::to_degrees(roll) << ",pitch:" <<
                angles::to_degrees(pitch) << ",yaw:" << angles::to_degrees(yaw) << std::endl;
            std::cout << "link roll:" << angles::to_degrees(linkRoll) << ",pitch:" <<
                angles::to_degrees(linkPitch) << ",yaw:" << angles::to_degrees(linkYaw) << std::endl;
#endif
		    tf2::Quaternion orientation;
		    orientation.setRPY(linkRoll - roll, linkPitch - pitch, linkYaw - yaw);
            alignedMsg.orientation = tf2::toMsg(orientation);
        }

        // transform to tcp frame if there is
        Eigen::Isometry3d transformed;
        Eigen::fromMsg(alignedMsg, transformed);
        if (world_to_tcp_)
        {
            // the position
            tf2::doTransform(transformed, transformed, *world_to_tcp_);
            for (size_t i = 0; i < 3; ++i)
            {
                transformed.matrix()(i, 3) = transformed.matrix()(i, 3) - position_reference_[i];
            }
            alignedMsg.position = Eigen::toMsg(transformed).position;

            // the orientation
            Eigen::Isometry3d transEuler(transformed);
            tf2::Quaternion q(alignedMsg.orientation.x, alignedMsg.orientation.y,
                alignedMsg.orientation.z, alignedMsg.orientation.w);
            double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            transEuler.matrix()(0, 3) = roll;
            transEuler.matrix()(1, 3) = pitch;
            transEuler.matrix()(2, 3) = yaw;
            tf2::doTransform(transEuler, transEuler, *object_to_tcp_);
            tf2::Quaternion orientation;
		    orientation.setRPY(transEuler.matrix()(0, 3), transEuler.matrix()(1, 3), transEuler.matrix()(2, 3));
            alignedMsg.orientation = tf2::toMsg(orientation);
        }

        if (pub_pose_)
        {
            whi_interfaces::WhiTcpPose msg;
            msg.tcp_pose.header.frame_id = pose_frame_;
            msg.tcp_pose.header.stamp = ros::Time::now();
            msg.tcp_pose.pose = alignedMsg;

            scalingEuler(msg.tcp_pose.pose.orientation, euler_multipliers_);

            pub_pose_->publish(msg);
        }
        if (client_pose_ && service_standby_.load())
        {
            std::thread
            {
                [this, alignedMsg]() -> void
                {
                    this->service_standby_.store(false);

                    if (this->client_pose_->waitForExistence(ros::Duration(3.0)))
				    {
                        whi_interfaces::WhiSrvTcpPose srv;
                        srv.request.pose.tcp_pose.header.frame_id = this->pose_frame_;
                        srv.request.pose.tcp_pose.header.stamp = ros::Time(0);
                        srv.request.pose.tcp_pose.pose = alignedMsg;

                        scalingEuler(srv.request.pose.tcp_pose.pose.orientation, euler_multipliers_);
#ifdef DEBUG
                        srv.request.pose.tcp_pose.pose.orientation.x = 0.0;
                        srv.request.pose.tcp_pose.pose.orientation.y = 0.0;
                        srv.request.pose.tcp_pose.pose.orientation.z = 0.0;
                        srv.request.pose.tcp_pose.pose.orientation.w = 1.0;
#endif
                        this->client_pose_->call(srv);
				    }
                    else
                    {
                        ROS_WARN_STREAM("failed to get the tcp_pose service, please check if it is launched");
                    }

                    this->service_standby_.store(true);
                }
            }.detach();
        }
    }

    void TriDObjectTracking::colorImageCallback(const std::string& Name, const cv::Mat& Image)
    {
        publishImage(pub_color_, Name, Image, sensor_msgs::image_encodings::BGR8, seq_++);
    }

    void TriDObjectTracking::colorOverlayImageCallback(const std::string& Name, const cv::Mat& Image)
    {
        publishImage(pub_color_overlay_, Name, Image, sensor_msgs::image_encodings::BGR8, seq_);
    }

    void TriDObjectTracking::depthImageCallback(const std::string& Name, const cv::Mat& Image)
    {
        publishImage(pub_depth_, Name, Image, "passthrough", seq_);
    }

    void TriDObjectTracking::depthOverlayImageCallback(const std::string& Name, const cv::Mat& Image)
    {
        publishImage(pub_depth_overlay_, Name, Image, sensor_msgs::image_encodings::BGR8, seq_);
    }

    void TriDObjectTracking::toImageMsg(const cv::Mat& SrcImg, const std::string& SrcEncoding,
        sensor_msgs::Image& RosImage)
    {
        RosImage.height = SrcImg.rows;
        RosImage.width = SrcImg.cols;
        RosImage.encoding = SrcEncoding;
        RosImage.is_bigendian = (boost::endian::order::native == boost::endian::order::big);
        RosImage.step = SrcImg.cols * SrcImg.elemSize();
        size_t size = RosImage.step * SrcImg.rows;
        RosImage.data.resize(size);

        if (SrcImg.isContinuous())
        {
            memcpy((char*)(&RosImage.data[0]), SrcImg.data, size);
        }
        else
        {
            // copy by row by row
            uchar* rosDataPtr = (uchar*)(&RosImage.data[0]);
            uchar* dataPtr = SrcImg.data;
            for (int i = 0; i < SrcImg.rows; ++i)
            {
                memcpy(rosDataPtr, dataPtr, RosImage.step);
                rosDataPtr += RosImage.step;
                dataPtr += SrcImg.step;
            }
        }
    }

    void TriDObjectTracking::publishImage(std::shared_ptr<image_transport::Publisher> Publisher,
        const std::string& Name, const cv::Mat& Image, const std::string& Encoding, unsigned long Seq/* = 0*/)
    {
        if (Publisher)
        {
            sensor_msgs::Image imgMsg;
            imgMsg.header.seq = Seq;
            imgMsg.header.stamp = ros::Time::now();
            imgMsg.header.frame_id = Name;
            toImageMsg(Image, Encoding, imgMsg);
            Publisher->publish(imgMsg);
        }
    }

    void TriDObjectTracking::toggleRightAndLeftHand(const Eigen::Isometry3d& Src, Eigen::Isometry3d& Dst)
    {
        Dst = Src;
        Dst.matrix()(0, 1) = Src.matrix()(0, 2);
        Dst.matrix()(0, 2) = Src.matrix()(0, 1);
        Dst.matrix()(1, 0) = Src.matrix()(2, 0);
        Dst.matrix()(1, 1) = Src.matrix()(2, 2);
        Dst.matrix()(1, 2) = Src.matrix()(2, 1);
        Dst.matrix()(2, 0) = Src.matrix()(1, 0);
        Dst.matrix()(2, 1) = Src.matrix()(1, 2);
        Dst.matrix()(2, 2) = Src.matrix()(1, 1);
        Dst.matrix()(1, 3) = Src.matrix()(2, 3);
        Dst.matrix()(2, 3) = Src.matrix()(1, 3);
        Dst.matrix()(3, 0) = 0.0;
        Dst.matrix()(3, 1) = 0.0;
        Dst.matrix()(3, 2) = 0.0;
        Dst.matrix()(3, 3) = 1.0;
    }

    void TriDObjectTracking::scalingEuler(geometry_msgs::Quaternion& Src, const std::array<double, 3>& Multiplier)
    {
        tf2::Quaternion q(Src.x, Src.y, Src.z, Src.w);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
#ifdef DEBUG
        std::cout << "original roll:" << angles::to_degrees(roll) << ",pitch:" <<
            angles::to_degrees(pitch) << ",yaw:" << angles::to_degrees(yaw) << std::endl;
#endif
        q.setRPY(roll * Multiplier[0], pitch * Multiplier[1], yaw * Multiplier[2]);
        Src = tf2::toMsg(q);
#ifdef DEBUG
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std::cout << "after roll:" << angles::to_degrees(roll) << ",pitch:" <<
            angles::to_degrees(pitch) << ",yaw:" << angles::to_degrees(yaw) << std::endl;
#endif
    }

    bool TriDObjectTracking::retrieveTransform(std::shared_ptr<ros::NodeHandle> Node,
        const std::string ParamName, geometry_msgs::TransformStamped& Trans)
    {
        std::vector<double> contents;
        if (Node && Node->getParam(ParamName, contents))
        {
            tf2::Quaternion q;
            if (contents.size() == 6)
            {
                q.setRPY(contents[3], contents[4], contents[5]);
            }
            else
            {
                q.setRPY(0.0, 0.0, 0.0);
            }
            
            tf2::Stamped<tf2::Transform> stamped;
            if (contents.size() >= 3)
            {
                stamped.setData(tf2::Transform(q, tf2::Vector3(contents[0], contents[1], contents[2])));
            }
            else
            {
                stamped.setData(tf2::Transform(q, tf2::Vector3(0.0, 0.0, 0.0)));
            }

            Trans = tf2::toMsg(stamped);
            return true;
        }

        return false;
    }
} // namespace whi_3DObjectTracking
