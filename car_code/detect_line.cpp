#include <ros/ros.h>
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include <sstream> // for converting the command line parameter to integer  
#include<vector>
#include "learning_image_transport/AddTwoInts.h"


using namespace cv;
using namespace std;

int  offsety=0;
int  offsetx=0;
int  blacklinetype=0;
int  black_k=0;

struct blackline{
   int x1;
   int y1;
   float b;
   float x0;
   float k;
   int type;
   int xc;
   int yc;
   float len;
}btmp,bline[30];


bool add(learning_image_transport::AddTwoInts::Request  &req,
         learning_image_transport::AddTwoInts::Response &res )
{
  res.result[0] = req.a;
  res.result[1] = blacklinetype;
  res.result[2] = offsetx;
  res.result[3] = offsety;
  res.result[4] = black_k;
  
  ROS_INFO("request: x=%d", (int)req.a);
  ROS_INFO("sending back response: [%d]", (int)res.result[1]);
  return true;
}


void line_cal(int x1,int y1,int x2,int y2)
{
    btmp.len=int(sqrt((y2-y1)*(y2-y1)+(x2-x1)*(x2-x1)));
    btmp.type=0;
    btmp.k=0;
    btmp.b=0;
    btmp.x0=0;
    if(abs(x2-x1)<2)
    {
        btmp.type=1;
        btmp.x0=x1;
        btmp.k=-1000000;
    }
    else
    {
        btmp.k=1.0*(y2-y1)/(x2-x1);
        btmp.b=1.0*(y1-x1*(y2-y1)/(x2-x1));
        if(abs(btmp.k)>480.0/640)
        {
            btmp.type=1;
            btmp.x0 =1.0*(x1-y1*(x2-x1)/(y2-y1));
        }
    }
}


int main(int argc, char** argv)  
{  
    // Check if video source has been passed as a parameter  
    if(argv[1] == NULL)   
    {  
        ROS_INFO("argv[1]=NULL\n");  
        // return 1;  
    }  

    ros::init(argc, argv, "image_publisher");  
    ros::NodeHandle nh; 
    ros::NodeHandle n; 
    image_transport::ImageTransport it(nh);  
    image_transport::Publisher pub = it.advertise("camera/image", 1);  

    ros::ServiceServer service = n.advertiseService("line_detect", add);


    ros::Rate loop_rate(15);  

    // Convert the passed as command line parameter index for the video device to an integer  
    // std::istringstream video_sourceCmd(argv[1]);  
    int video_source;  
    // Check if it is indeed a number  
    /*
    if(!(video_sourceCmd >> video_source))   
    {  
        ROS_INFO("video_sourceCmd is %d\n",video_source);  
        return 1;  
    } 
    */ 

    cv::VideoCapture cap(10);  
    // Check if video device can be opened with the given index  
    if(!cap.isOpened())   
    {  
        ROS_INFO("can not opencv video device\n");  
        return 1;  
    }  
    ROS_INFO("opencv video device\n");  
    cv::Mat frame;  
    sensor_msgs::ImagePtr msg;
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  
    cv::Mat newimage;
    struct timeval tpstart,tpend;
    float timeuse;
    Mat edges;
    vector<Vec4i>lines;
    int line_num_h=0;
    int line_num_v=0;

    while (nh.ok()) 
    {  
        cap >> frame;  
        // Check if grabbed frame is actually full with some content  
        if(!frame.empty()) 
        {  

    line_num_h=0;
    line_num_v=0;

    gettimeofday(&tpstart,NULL);
    cv::resize(frame,newimage,Size(640,480));
    Mat gray,DestImageOut;
    cv::cvtColor(newimage,gray, COLOR_RGB2GRAY);

    cv::GaussianBlur(gray,gray,Size(15,15),0);

    Canny(gray,edges,40, 120,3);
    HoughLinesP(edges,lines,1, CV_PI/360, 60,150,240);

    for( size_t i = 0; i < lines.size(); i++ )
     {
       Vec4i l = lines[i];
       line_cal(l[0],l[1],l[2],l[3]);
       if(btmp.type==0)
       {
           if(line_num_h==0)
           {
             bline[0].b=btmp.b;bline[0].k=btmp.k;bline[0].type=btmp.type;bline[0].x0=btmp.x0;
             line_num_h++;
           }
           else
           {
             int j;
             for(j=0;j<line_num_h;j++)
             {
                if(abs(bline[j].b-btmp.b)<8)
                {
                    break;
                }
             }
             if(j>=line_num_h)
             {
		if(line_num_h>5)break;
                bline[j].b=btmp.b;bline[j].k=btmp.k;bline[j].type=btmp.type;bline[j].x0=btmp.x0;
                line_num_h++;
             }
           }
       }
       else
       {
           if(line_num_v==0)
           {
             bline[10].b=btmp.b;bline[10].k=btmp.k;bline[10].type=btmp.type;bline[10].x0=btmp.x0;
             line_num_v++;
           }
           else
           {
             int j;
             for(j=0;j<line_num_v;j++)
             {  
		if(line_num_v>5)break;
                if(abs(bline[10+j].x0-btmp.x0)<8)
                {
                    break;
                }
             }
             if(j>=line_num_v)
             {
                bline[10+j].b=btmp.b;bline[10+j].k=btmp.k;bline[10+j].type=btmp.type;bline[10+j].x0=btmp.x0;
                line_num_v++;
             }
           }
       }
     }
    int rl=0;
    int xc=0;
    int yc=0;
    if(line_num_h>=1)
    {
        rl++;
        if(line_num_h==1)
        {
            yc=240-bline[0].b+bline[0].k*320;
        }
        else
        {
            yc=240-((bline[0].b+bline[0].k*320)+(bline[1].b+bline[1].k*320))/2;
//          qDebug()<<(bline[0].b+bline[0].k*320)<<(bline[1].b+bline[1].k*320);
        }
        black_k = bline[0].k*100;

    }
    if(line_num_v>=1)
    {
        rl+=2;
        if(line_num_v==1)
        {
            xc=320-240/bline[10].k+bline[10].x0;
        }
        else
        {
            xc=320-(240/bline[10].k+bline[10].x0+240/bline[11].k+bline[11].x0)/2;

        }
    }



    offsety=yc;
    offsetx=xc;
    gettimeofday(&tpend,NULL);
    timeuse=(1000000*(tpend.tv_sec-tpstart.tv_sec) + tpend.tv_usec-tpstart.tv_usec)/1000000.0;
 
    ROS_INFO("time:%f,type:%d,cx=%d,cy=%d,lm:=%d,k=%d\n",timeuse,rl,offsetx,offsety,line_num_v+line_num_h,black_k);  

    blacklinetype=rl;
    if(rl==1)
        line(newimage, Point(0,240-offsety), Point(640,240-offsety), Scalar(0, 255, 255), 2);
    else if(rl==2)
        line(newimage, Point(320-offsetx,0), Point(320-offsetx,480), Scalar(0, 255, 255), 2);
    else if(rl==3)
    {
        line(newimage, Point(0,240-offsety), Point(640,240-offsety), Scalar(0, 255, 255), 2);
        line(newimage, Point(320-offsetx,0), Point(320-offsetx,480), Scalar(0, 255, 255), 2);
    }

//          msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", newimage).toImageMsg();  
            pub.publish(msg);
            //cv::Wait(1);
    	}
    }
    
    ros::spin();
    loop_rate.sleep();
}
