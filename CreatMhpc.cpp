//本程序实现需要用到三方面的知识：openCV2.4.9; KinectV2.0; PCL1.7.2 。    
//openCV2.4.9主要用其循环获取每张16位深度图像，并遍历每个像素点获取深度值，赋给KinectV2.0用于坐标转化，即为三维数据里的深度值。
//KinectV2.0 主要用到SDK内在的坐标转化（双目摄像头：彩色（只需知其大小即可1920*1080）和深度（由openCV2.4.9获取其大小和深度值512*424）。），转化可得相机坐标系下目标的真实三维数据。
//PCL1.7.2主要用到点云的数据存储格式，将KinectV2.0 转化出来的相机坐标系下的每个三维点，以及所需颜色值存入.pcd内。


//*************利用白色背景的话，可以不生成图片进行保存，因为遍历每个像素进行修改之后，得到的正好是同师姐一样的数据库，可以直接利用那个Mat 类型
//10月31日 改动，将点云文件保存为二进制形式，节省了时间和空间

#include<afx.h>   //1007
#define NOMINMAX
#include <iostream>
#include<time.h>
#include <windows.h>
#include <cstring>    //1007
#include <Kinect.h>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <pcl/point_types.h>
//#include <pcl/filters/statistical_outlier_removal.h>    //点云滤波
using namespace cv;
using namespace std;


template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
{
    if( pInterfaceToRelease != NULL ){
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
    }
}


//look up the filename     1007
int CountDirectory(CString path)
{
	int count = 0;
	CFileFind finder;
	BOOL working = finder.FindFile(path + "\\*.png");
	while (working)
	{
		working = finder.FindNextFile();
		if (finder.IsDots())
			continue;
		if (finder.IsDirectory())
			count += CountDirectory(finder.GetFilePath());
		else
			count++;
	}

	return count;
}

//将小图像进行扩展
 void extendImg(Mat & inputImage,Mat & outputImage )
{
	//方法二  利用.at
	for(int i=0;i<240;i++)   //  第一种方法，有问题
	{
		for(int j=0;j<320;j++)
		{
			if(inputImage.at<uint16_t>(i,j)!=0)
			{
				outputImage.at<uint16_t>(i+82,j+96)=inputImage.at<uint16_t>(i,j);

			}
		}
	}
}
int _tmain(int argc, _TCHAR* argv[])
{
	//openCV2.4.9循环获取每张深度图
	char filename[100];   //设置最大图像地址（filename）序列为10000
	//PCL1.7.2
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloudss( new pcl::PointCloud<pcl::PointXYZRGB>() );       /// save  Mhpc,it must  be put in the loop
	clock_t time_start,time_end;

	CString pathin;   //1007
	pathin = "F:\\MSR action 3D 实验\\修改后的数据库1";   //1007
	CFileFind Finder;
	BOOL Working = Finder.FindFile(pathin+"\\*sdepth");  // ?
	//Working = Finder.FindNextFile();
	while (Working) 
	{
	    Working = Finder.FindNextFile();  //?
	    
		//KinectV2.0 获取传感器， 并打开，然后获取坐标映射
		// Create Sensor Instance
		IKinectSensor* pSensor;                      //创建传感器实例
		HRESULT hResult = S_OK;                      //函数返回值，用来判断函数运行成功否
		hResult = GetDefaultKinectSensor( &pSensor );  //得到默认 Kinect 传感器
		if( FAILED( hResult ) ){
			std::cerr << "Error : GetDefaultKinectSensor" << std::endl;
			return -1;
		}

		// Open Sensor
		hResult = pSensor->Open();
		if( FAILED( hResult ) ){
			std::cerr << "Error : IKinectSensor::Open()" << std::endl;
			return -1;
		}

		// Retrieved Coordinate Mapper 
		ICoordinateMapper* pCoordinateMapper;
		hResult = pSensor->get_CoordinateMapper( &pCoordinateMapper );         
		if( FAILED( hResult ) ){
			std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
			return -1;
		}

		int depthWidth = 512;      //设置深度图像的大小
		int depthHeight = 424;

		int colorWidth = 1920;     //设置彩色图像的大小
		int colorHeight = 1080;

		//PCL1.7.2
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloudss( new pcl::PointCloud<pcl::PointXYZRGB>() );       /// save  Mhpc,it must  be put in the loop

		int num;
		char path[100],title[100],title_picture[100]; //to save the path variate
		sprintf_s(path, "F:\\MSR action 3D 实验\\修改后的数据库1\\%s", (LPCSTR)Finder.GetFileName());   //  1007

		num=CountDirectory(path);    //  1007
		int k=1; //0901
		for ( int i =0;i <num; i=i++)  //循环获取每帧深度图像   每个采集的帧数是不一样的，所以每一个数据都要改 !!!!!1  0720//0
		{
		
			Mat img( 240, 320, CV_16UC1 );
			Mat img_extend( depthHeight, depthWidth, CV_16UC1,Scalar(65535));
			//获取第i帧深度图像
			sprintf_s(title_picture, "%s", (LPCSTR)Finder.GetFilePath());
			sprintf_s( filename,"%s\\%02d.png",title_picture, i );  // !!!!!!!2    //  1007  ?
			img = imread( filename,CV_LOAD_IMAGE_ANYDEPTH); 
			//namedWindow( "initial img", 2 );  
			//imshow("initial img",img);

			//将图像进行扩充 240×160变成 512×424
			 extendImg(img,img_extend);
			//namedWindow( "extended img", CV_WINDOW_AUTOSIZE);  //改为自动调节大小，看看能否显示16位的图
			//imshow("extended img", img_extend );
			cout<<k<<endl;
			cout<<"读取放大完毕"<<endl;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointclouds(new pcl::PointCloud<pcl::PointXYZRGB>());       ///创建点云指针对象--存储每幅运动物体点云
			time_start = clock();		
			//遍历每个像素点并使用KinectV2.0结合深度、彩色图像转换为相机坐标系下的三位点，存储到每幅运动物体点云里面
			for( int y = 0; y < depthHeight; y++ )
			{
				for( int x = 0; x < depthWidth; x++ )
				{
					pcl::PointXYZRGB point;
					DepthSpacePoint depthSpacePoint = { static_cast<float>( x ), static_cast<float>( y ) };         //定义KinectV2.0深度空间点
				  // if(img_extend.at<UINT16>( y, x)!=65535)
					//{
						UINT16 depth = img_extend.at<UINT16>( y, x);       //利用滤波之后的深度图像     0906                 //将深度图像像素值赋给16位depth
					 //UINT16 depth = img.at<UINT16>( y, x);
					 unsigned int index = y * depthWidth + x;                                                     //像素点索引序列index

					// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
					ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };                                              //定义KinectV2.0彩色空间点
					pCoordinateMapper->MapDepthPointToColorSpace( depthSpacePoint, depth, &colorSpacePoint );      //将深度空间点映射到彩色空间
					int colorX = static_cast<int>( std::floor( colorSpacePoint.X + 0.5f ) );                       //彩色空间X值
					int colorY = static_cast<int>( std::floor( colorSpacePoint.Y + 0.5f ) );                       //彩色空间Y值

					if (depth < 3000)                                                                              //设置深度值阈值，根据运动目标距摄像头距离设置，可使前景提取，背景去除
					{
						if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) )
						 {
							point.b = 0;                                                                           //设置点云蓝色通道值0
							point.g =2*( i+1);            //1024                                                               //设置点云绿色通道值 ，使其根据帧数递加
							point.r = 0;                                                                           //设置点云红色通道值0
						 }

						// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
						CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };                                  //定义KinectV2.0相机空间点
						pCoordinateMapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint );  //将深度空间点映射到相机空间，此时点的值为相机坐标系下的值，即目标真实值大小
						if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) )
						 {
							point.x = cameraSpacePoint.X;
							point.y = cameraSpacePoint.Y;                                                           //将相机坐标系下三维点的X值赋给点云三维点的x
							point.z = cameraSpacePoint.Z;                                                           //将相机坐标系下三维点的Y值赋给点云三维点的y
       
						 }                                                                                           //将相机坐标系下三维点的Z值赋给点云三维点的z
 
						 pointclouds->push_back( point );    ///        //将三维点添加到点云里				  				 
					 }  
				//}
			  }
			}
			// k++;
			time_end = clock();
			double time_total = time_end-time_start;
			cout << "运行时间："<<time_total/CLOCKS_PER_SEC<<endl;

			*pointcloudss += *pointclouds;         /// 运动历史点云 += 每幅运动物体点云；
			cout << "运动历史帧数" << i << " " << "运动历史点云数pointcloudss = " << pointcloudss->points.size()<<endl;       ///可删除
          
			/*if ( waitKey(30) >=27 )
			{
				break;
			}*/

			//加入到注释里面  0905
		}   

		 //保存运动历史点云
		//sprintf_s(title, "F://MSR action 3D 实验//实验得到的数据//MHPC//%s_mhpc.pcd", (LPCSTR)Finder.GetFileName()); 
		sprintf_s(title, "F://MSR action 3D 实验//运动历史点云4//%s_mhpc.pcd", (LPCSTR)Finder.GetFileName());
		cout << title <<endl;
		//sprintf_s(filename1,"%s\\%02d.png", title_out,f);//保存路径设置
		pcl::io::savePCDFileBinary(title, *pointcloudss);   //!!!!!!!! 3
		cout<<"存储完毕！"<<endl;
		 //可视化运动历史点云
		//节省时间将点云可视化去除
	   /* pcl::visualization::CloudViewer viewer( "Point Cloud Viewer" );
		viewer.showCloud( pointcloudss ); 
		while (!viewer.wasStopped ())
		{
		}*/
	
	//下面的关于Kinect的指针不知道是不是也要放到循环里面

    // End Processing释放转换器指针
    SafeRelease( pCoordinateMapper );                                                                                                               
    if( pSensor )
	{
        pSensor->Close();            //关闭Kinect传感器
    }
    SafeRelease( pSensor );          //释放Kinect传感器指针

   }

    return 0;
}