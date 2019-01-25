#include <opencv2/viz.hpp>
#include <iostream>
#include <fstream>

//using namespace cv;

class PointCloudVisualizer
{
    public:
        bool camera_pov = true;
        cv::Mat cloud;
        cv::Vec3f cam_pos, cam_focal_point, cam_y_dir;
        std::vector<cv::Point3f> points_buffer;

        PointCloudVisualizer()
        {
            cam_pos = cv::Vec3f(3.0f,3.0f,3.0f);
            cam_focal_point = cv::Vec3f(3.0f,3.0f,2.0f);
            cam_y_dir = cv::Vec3f(-1.0f,0.0f,0.0f);

        }

        template<class T>
        void addPoint(T data)
        {
            points_buffer.push_back(cv::Point3f(data.x,data.y,data.z));
        }


        void commitPoints()
        {
            cloud.create(1, points_buffer.size(), CV_32FC3);
            for(uint i = 0 ; i < points_buffer.size() ; i++)
            {
                cv::Point3f* data = cloud.ptr<cv::Point3f>();
                data[i].x = points_buffer[i].x;
                data[i].y = points_buffer[i].y;
                data[i].z = points_buffer[i].z;
            }
            std::cout<<points_buffer.size()<<" of points commmited to pcv."<<std::endl;
            points_buffer.clear();
        }

        void setSource(std::string filename)
        {
            //Mat cloud(1, 1889, CV_32FC3);
            std::ifstream ifs(filename);

            std::string str;
            for(size_t i = 0; i < 13; ++i)
            {
                getline(ifs, str);
                //cout<<"str = "<<str<<endl;
            }
            //Point3f* data = cloud.ptr<cv::Point3f>();
            float dummy1, dummy2;
            std::vector<cv::Point3f> points;
            //while(!ifs.eof())
            for(size_t i = 0; i < 10000; ++i)
            {
                cv::Point3f p;
                ifs >> p.x >> p.y >> p.z >> dummy1 >> dummy2;
                //cout<<"p = "<<p<<endl;
                //cout<<"dummy1 = "<<dummy1<<endl;
                //cout<<"dummy2 = "<<dummy2<<endl;
                points.push_back(p);
            }
            cloud.create(1, points.size(), CV_32FC3);
            for(uint i = 0 ; i < points.size() ; i++)
            {
                cv::Point3f* data = cloud.ptr<cv::Point3f>();
                data[i].x = points[i].x;
                data[i].y = points[i].y;
                data[i].z = points[i].z;
            }

            //cloud *= 5.0f;
            //return cloud;

        }

        void setCamPov(bool in)
        {
            camera_pov = in;
        }

        void show()
        {
            /// Create a window
            cv::viz::Viz3d myWindow("Coordinate Frame");

            /// Add coordinate axes
            myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

            /// Let's assume camera has the following properties
            cv::Vec3f cam_pos(3.0f,3.0f,3.0f), cam_focal_point(3.0f,3.0f,2.0f), cam_y_dir(-1.0f,0.0f,0.0f);

            /// We can get the pose of the cam using makeCameraPose
            cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

            /// We can get the transformation matrix from camera coordinate system to global using
            /// - makeTransformToGlobal. We need the axes of the camera
            cv::Affine3f transform = cv::viz::makeTransformToGlobal(cv::Vec3f(0.0f,-1.0f,0.0f), cv::Vec3f(-1.0f,0.0f,0.0f), cv::Vec3f(0.0f,0.0f,-1.0f), cam_pos);

            /// Create a cloud widget.
            //Mat bunny_cloud = cvcloud_load();
            cv::viz::WCloud cloud_widget(cloud, cv::viz::Color::green());

            /// Pose of the widget in camera frame
            cv::Affine3f cloud_pose = cv::Affine3f().translate(cv::Vec3f(0.0f,0.0f,3.0f));
            /// Pose of the widget in global frame
            cv::Affine3f cloud_pose_global = transform * cloud_pose;

            /// Visualize camera frame
            if (!camera_pov)
            {
                cv::viz::WCameraPosition cpw(0.5); // Coordinate axes
                cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599)); // Camera frustum
                myWindow.showWidget("CPW", cpw, cam_pose);
                myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
            }

            /// Visualize widget
            myWindow.showWidget("bunny", cloud_widget, cloud_pose_global);

            /// Set the viewer pose to that of camera
            if (camera_pov)
                myWindow.setViewerPose(cam_pose);

            /// Start event loop.
            myWindow.spin();


        }

};
