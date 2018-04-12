//������ʵ����Ҫ�õ��������֪ʶ��openCV2.4.9; KinectV2.0; PCL1.7.2 ��    
//openCV2.4.9��Ҫ����ѭ����ȡÿ��16λ���ͼ�񣬲�����ÿ�����ص��ȡ���ֵ������KinectV2.0��������ת������Ϊ��ά����������ֵ��
//KinectV2.0 ��Ҫ�õ�SDK���ڵ�����ת����˫Ŀ����ͷ����ɫ��ֻ��֪���С����1920*1080������ȣ���openCV2.4.9��ȡ���С�����ֵ512*424��������ת���ɵ��������ϵ��Ŀ�����ʵ��ά���ݡ�
//PCL1.7.2��Ҫ�õ����Ƶ����ݴ洢��ʽ����KinectV2.0 ת���������������ϵ�µ�ÿ����ά�㣬�Լ�������ɫֵ����.pcd�ڡ�


//*************���ð�ɫ�����Ļ������Բ�����ͼƬ���б��棬��Ϊ����ÿ�����ؽ����޸�֮�󣬵õ���������ͬʦ��һ�������ݿ⣬����ֱ�������Ǹ�Mat ����
//10��31�� �Ķ����������ļ�����Ϊ��������ʽ����ʡ��ʱ��Ϳռ�

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
//#include <pcl/filters/statistical_outlier_removal.h>    //�����˲�
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

//��Сͼ�������չ
 void extendImg(Mat & inputImage,Mat & outputImage )
{
	//������  ����.at
	for(int i=0;i<240;i++)   //  ��һ�ַ�����������
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
	//openCV2.4.9ѭ����ȡÿ�����ͼ
	char filename[100];   //�������ͼ���ַ��filename������Ϊ10000
	//PCL1.7.2
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloudss( new pcl::PointCloud<pcl::PointXYZRGB>() );       /// save  Mhpc,it must  be put in the loop
	clock_t time_start,time_end;

	CString pathin;   //1007
	pathin = "F:\\MSR action 3D ʵ��\\�޸ĺ�����ݿ�1";   //1007
	CFileFind Finder;
	BOOL Working = Finder.FindFile(pathin+"\\*sdepth");  // ?
	//Working = Finder.FindNextFile();
	while (Working) 
	{
	    Working = Finder.FindNextFile();  //?
	    
		//KinectV2.0 ��ȡ�������� ���򿪣�Ȼ���ȡ����ӳ��
		// Create Sensor Instance
		IKinectSensor* pSensor;                      //����������ʵ��
		HRESULT hResult = S_OK;                      //��������ֵ�������жϺ������гɹ���
		hResult = GetDefaultKinectSensor( &pSensor );  //�õ�Ĭ�� Kinect ������
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

		int depthWidth = 512;      //�������ͼ��Ĵ�С
		int depthHeight = 424;

		int colorWidth = 1920;     //���ò�ɫͼ��Ĵ�С
		int colorHeight = 1080;

		//PCL1.7.2
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloudss( new pcl::PointCloud<pcl::PointXYZRGB>() );       /// save  Mhpc,it must  be put in the loop

		int num;
		char path[100],title[100],title_picture[100]; //to save the path variate
		sprintf_s(path, "F:\\MSR action 3D ʵ��\\�޸ĺ�����ݿ�1\\%s", (LPCSTR)Finder.GetFileName());   //  1007

		num=CountDirectory(path);    //  1007
		int k=1; //0901
		for ( int i =0;i <num; i=i++)  //ѭ����ȡÿ֡���ͼ��   ÿ���ɼ���֡���ǲ�һ���ģ�����ÿһ�����ݶ�Ҫ�� !!!!!1  0720//0
		{
		
			Mat img( 240, 320, CV_16UC1 );
			Mat img_extend( depthHeight, depthWidth, CV_16UC1,Scalar(65535));
			//��ȡ��i֡���ͼ��
			sprintf_s(title_picture, "%s", (LPCSTR)Finder.GetFilePath());
			sprintf_s( filename,"%s\\%02d.png",title_picture, i );  // !!!!!!!2    //  1007  ?
			img = imread( filename,CV_LOAD_IMAGE_ANYDEPTH); 
			//namedWindow( "initial img", 2 );  
			//imshow("initial img",img);

			//��ͼ��������� 240��160��� 512��424
			 extendImg(img,img_extend);
			//namedWindow( "extended img", CV_WINDOW_AUTOSIZE);  //��Ϊ�Զ����ڴ�С�������ܷ���ʾ16λ��ͼ
			//imshow("extended img", img_extend );
			cout<<k<<endl;
			cout<<"��ȡ�Ŵ����"<<endl;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointclouds(new pcl::PointCloud<pcl::PointXYZRGB>());       ///��������ָ�����--�洢ÿ���˶��������
			time_start = clock();		
			//����ÿ�����ص㲢ʹ��KinectV2.0�����ȡ���ɫͼ��ת��Ϊ�������ϵ�µ���λ�㣬�洢��ÿ���˶������������
			for( int y = 0; y < depthHeight; y++ )
			{
				for( int x = 0; x < depthWidth; x++ )
				{
					pcl::PointXYZRGB point;
					DepthSpacePoint depthSpacePoint = { static_cast<float>( x ), static_cast<float>( y ) };         //����KinectV2.0��ȿռ��
				  // if(img_extend.at<UINT16>( y, x)!=65535)
					//{
						UINT16 depth = img_extend.at<UINT16>( y, x);       //�����˲�֮������ͼ��     0906                 //�����ͼ������ֵ����16λdepth
					 //UINT16 depth = img.at<UINT16>( y, x);
					 unsigned int index = y * depthWidth + x;                                                     //���ص���������index

					// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
					ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };                                              //����KinectV2.0��ɫ�ռ��
					pCoordinateMapper->MapDepthPointToColorSpace( depthSpacePoint, depth, &colorSpacePoint );      //����ȿռ��ӳ�䵽��ɫ�ռ�
					int colorX = static_cast<int>( std::floor( colorSpacePoint.X + 0.5f ) );                       //��ɫ�ռ�Xֵ
					int colorY = static_cast<int>( std::floor( colorSpacePoint.Y + 0.5f ) );                       //��ɫ�ռ�Yֵ

					if (depth < 3000)                                                                              //�������ֵ��ֵ�������˶�Ŀ�������ͷ�������ã���ʹǰ����ȡ������ȥ��
					{
						if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) )
						 {
							point.b = 0;                                                                           //���õ�����ɫͨ��ֵ0
							point.g =2*( i+1);            //1024                                                               //���õ�����ɫͨ��ֵ ��ʹ�����֡���ݼ�
							point.r = 0;                                                                           //���õ��ƺ�ɫͨ��ֵ0
						 }

						// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
						CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };                                  //����KinectV2.0����ռ��
						pCoordinateMapper->MapDepthPointToCameraSpace( depthSpacePoint, depth, &cameraSpacePoint );  //����ȿռ��ӳ�䵽����ռ䣬��ʱ���ֵΪ�������ϵ�µ�ֵ����Ŀ����ʵֵ��С
						if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) )
						 {
							point.x = cameraSpacePoint.X;
							point.y = cameraSpacePoint.Y;                                                           //���������ϵ����ά���Xֵ����������ά���x
							point.z = cameraSpacePoint.Z;                                                           //���������ϵ����ά���Yֵ����������ά���y
       
						 }                                                                                           //���������ϵ����ά���Zֵ����������ά���z
 
						 pointclouds->push_back( point );    ///        //����ά����ӵ�������				  				 
					 }  
				//}
			  }
			}
			// k++;
			time_end = clock();
			double time_total = time_end-time_start;
			cout << "����ʱ�䣺"<<time_total/CLOCKS_PER_SEC<<endl;

			*pointcloudss += *pointclouds;         /// �˶���ʷ���� += ÿ���˶�������ƣ�
			cout << "�˶���ʷ֡��" << i << " " << "�˶���ʷ������pointcloudss = " << pointcloudss->points.size()<<endl;       ///��ɾ��
          
			/*if ( waitKey(30) >=27 )
			{
				break;
			}*/

			//���뵽ע������  0905
		}   

		 //�����˶���ʷ����
		//sprintf_s(title, "F://MSR action 3D ʵ��//ʵ��õ�������//MHPC//%s_mhpc.pcd", (LPCSTR)Finder.GetFileName()); 
		sprintf_s(title, "F://MSR action 3D ʵ��//�˶���ʷ����4//%s_mhpc.pcd", (LPCSTR)Finder.GetFileName());
		cout << title <<endl;
		//sprintf_s(filename1,"%s\\%02d.png", title_out,f);//����·������
		pcl::io::savePCDFileBinary(title, *pointcloudss);   //!!!!!!!! 3
		cout<<"�洢��ϣ�"<<endl;
		 //���ӻ��˶���ʷ����
		//��ʡʱ�佫���ƿ��ӻ�ȥ��
	   /* pcl::visualization::CloudViewer viewer( "Point Cloud Viewer" );
		viewer.showCloud( pointcloudss ); 
		while (!viewer.wasStopped ())
		{
		}*/
	
	//����Ĺ���Kinect��ָ�벻֪���ǲ���ҲҪ�ŵ�ѭ������

    // End Processing�ͷ�ת����ָ��
    SafeRelease( pCoordinateMapper );                                                                                                               
    if( pSensor )
	{
        pSensor->Close();            //�ر�Kinect������
    }
    SafeRelease( pSensor );          //�ͷ�Kinect������ָ��

   }

    return 0;
}