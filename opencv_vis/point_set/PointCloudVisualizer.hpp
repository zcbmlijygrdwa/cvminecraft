#include <opencv2/viz.hpp>
#include <iostream>
#include <fstream>

//using namespace cv;

class PointCloudVisualizer
{
    private:
        bool is_color_set;
        bool camera_pov;
        int point_size;
    public:
        bool is_cloud_widget_set;
        cv::viz::WCloud* cloud_widget_ptr;
        cv::Mat colors;
        cv::Mat cloud;
        cv::Vec3f cam_pos, cam_focal_point, cam_y_dir;
        std::vector<cv::Point3f> points_buffer;
        std::vector<cv::Point3f> colors_buffer;
        int point_count;
        /// Create a window
        cv::viz::Viz3d myWindow;

        cv::Affine3f cloud_pose_global;
        cv::Affine3f cam_pose;

        void clearBuffers()
        {
            points_buffer.clear();
            colors_buffer.clear(); 
        }

        PointCloudVisualizer()
        {
            cam_pos = cv::Vec3f(3.0f,3.0f,3.0f);
            cam_focal_point = cv::Vec3f(3.0f,3.0f,2.0f);
            cam_y_dir = cv::Vec3f(-1.0f,0.0f,0.0f);
            point_count = 0;

            is_color_set = false;
            is_cloud_widget_set = false;
            camera_pov = true;

            clearBuffers();            

            cloud_widget_ptr = NULL;

            point_size = 2;
            /// Create a window
            myWindow = cv::viz::Viz3d("Coordinate Frame");

            /// Let's assume camera has the following properties
            cam_pos = cv::Vec3f(3.0f,3.0f,3.0f);
            cam_focal_point = cv::Vec3f(3.0f,3.0f,2.0f);
            cam_y_dir = cv::Vec3f(-1.0f,0.0f,0.0f);

            /// We can get the pose of the cam using makeCameraPose
            cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);

            /// We can get the transformation matrix from camera coordinate system to global using
            /// - makeTransformToGlobal. We need the axes of the camera
            cv::Affine3f transform = cv::viz::makeTransformToGlobal(cv::Vec3f(0.0f,-1.0f,0.0f), cv::Vec3f(-1.0f,0.0f,0.0f), cv::Vec3f(0.0f,0.0f,-1.0f), cam_pos);

            /// Pose of the widget in camera frame
            cv::Affine3f cloud_pose = cv::Affine3f().translate(cv::Vec3f(0.0f,0.0f,3.0f));
            /// Pose of the widget in global frame
            cloud_pose_global = transform * cloud_pose;

            /// Set the viewer pose to that of camera
            myWindow.setViewerPose(cam_pose);

        }


        template<class T>
            void setPoint(T data)
            {
                points_buffer = data;
            }

        template<class T>
            void setColorPoint(T data1,T data2)
            {
                points_buffer = data1;
                colors_buffer = data2;        
            }

        template<class T>
            void addPoint(T data)
            {
                points_buffer.push_back(cv::Point3f(data.x,data.y,data.z));
            }

        template<class T>
            void addColorPoint(T data,int r, int g, int b)
            {
                points_buffer.push_back(cv::Point3f(data.x,data.y,data.z));
                colors_buffer.push_back(cv::Point3f(r,g,b));        
            }

        template<class T>
            void addColorPoint(T x, T y, T z, int r, int g, int b)
            {
                points_buffer.push_back(cv::Point3f(x,y,z));
                colors_buffer.push_back(cv::Point3f(r,g,b));        
            }

        void commitPoints()
        {
            point_count = points_buffer.size();

            if(point_count==0)
            {
                return;
            }

            if(colors_buffer.size()==points_buffer.size())
            {
                std::cout<<"same color and point size.";
                is_color_set = true;
                colors.create(1, point_count, CV_8UC4);
                for(int i = 0 ; i < point_count;i++)
                {
                    cv::Vec4b* data = colors.ptr<cv::Vec4b>();
                    data[i].val[0] = colors_buffer[i].x;
                    data[i].val[1] = colors_buffer[i].y;
                    data[i].val[2] = colors_buffer[i].z;
                    data[i].val[3] = 255;
                }

                cloud.create(1, points_buffer.size(), CV_32FC3);
                for(uint i = 0 ; i < points_buffer.size() ; i++)
                {
                    cv::Point3f* data = cloud.ptr<cv::Point3f>();
                    data[i].x = points_buffer[i].x;
                    data[i].y = points_buffer[i].y;
                    data[i].z = points_buffer[i].z;
                }
                cloud_widget_ptr = new cv::viz::WCloud(cloud, colors);
            }
            else
            {
                is_color_set = false;
                std::cout<<"size of colors buffer and size of points buffer different. color not set.";

                cloud.create(1, points_buffer.size(), CV_32FC3);
                for(uint i = 0 ; i < points_buffer.size() ; i++)
                {
                    cv::Point3f* data = cloud.ptr<cv::Point3f>();
                    data[i].x = points_buffer[i].x;
                    data[i].y = points_buffer[i].y;
                    data[i].z = points_buffer[i].z;
                }
                cloud_widget_ptr = new cv::viz::WCloud(cloud, cv::viz::Color::green());
            }
            std::cout<<" | "<<points_buffer.size()<<" of points commmited to pcv."<<std::endl;
            cloud_widget_ptr->setRenderingProperty( cv::viz::POINT_SIZE, point_size);

        }


        void randomColor()
        {
            is_color_set = true;
            colors.create(1, point_count, CV_8UC4);
            for(int i = 0 ; i < point_count;i++)
            {
                cv::Vec4b* data = colors.ptr<cv::Vec4b>();
                data[i].val[0] = i%255;
                data[i].val[1] = 255-i%255;
                data[i].val[2] = (i*i*i)%255;
                data[i].val[3] = 255;
            }
            std::cout<<"color is set to random."<<std::endl;
        }

        void loadFromSource(std::string filename)
        {
            clearBuffers();

            std::ifstream ifs(filename);

            std::string str;
            for(size_t i = 0; i < 13; ++i)
            {
                getline(ifs, str);
                //cout<<"str = "<<str<<endl;
            }
            float dummy1, dummy2;
            std::vector<cv::Point3f> points;
            //while(!ifs.eof())
            for(size_t i = 0; i < 10000; ++i)
            {
                cv::Point3f p;
                ifs >> p.x >> p.y >> p.z >> dummy1 >> dummy2;
                points_buffer.push_back(p);
            }
        }

        void setCamPov(bool in)
        {
            camera_pov = in;
        }

        void setPointSize(int in)
        {
            point_size = in;
        }


        void addCameraFrame()
        {
            /// Visualize camera frame
            if (!camera_pov)
            {
                cv::viz::WCameraPosition cpw(0.5); // Coordinate axes
                cv::viz::WCameraPosition cpw_frustum(cv::Vec2f(0.889484, 0.523599)); // Camera frustum
                myWindow.showWidget("CPW", cpw, cam_pose);
                myWindow.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
            }
        }

        void show()
        {

            /// Add coordinate axes
            myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

            /// Visualize widget
            myWindow.showWidget("my_pointcloud", *cloud_widget_ptr, cloud_pose_global);
            is_cloud_widget_set = true;

            //std::cout<<"myWindow = "<<(&myWindow)<<std::endl;

            /// Start event loop.
            myWindow.spin();

        }

        void showOnce()
        {
            /// Add coordinate axes
            myWindow.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());

            if(is_cloud_widget_set)
            {
                myWindow.removeWidget("my_pointcloud");
            }

            /// Visualize widget
            myWindow.showWidget("my_pointcloud", *cloud_widget_ptr, cloud_pose_global);
            is_cloud_widget_set = true;

            myWindow.spinOnce(100,true);
        }

};
