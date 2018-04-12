#include<afx.h>   //1025
#include<iostream>
#include<opencv2/opencv.hpp>    //hog
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<vector>
#include <cstring>    //1025
#include<pcl/io/pcd_io.h>

using namespace std;
using namespace cv;

typedef unsigned short uint16_t;

//define normalize fuction
 void  normalize_x_z(vector<float> &a)
 {
    float max=*max_element(a.begin(),a.end()); 
   //<<endl;
    float min=*min_element(a.begin(),a.end());
 //  cout<<" min data = "<<min<<endl;
   //cout<<" differ  = "<<max-min<<endl;
// cout<<"max data = "<<max<<"            min data = "<<min<<"            differ  = "<<max-min<<endl;
// nomalize
 for(int f=0;f<a.size(); ++f)
	 {
		 a[f]=(511-0)*(a[f]-min)/(max-min)+0 ;
		//features[f] = (ymax-ymin)*(features[f]-dMinValue)/(dMaxValue-dMinValue+1e-8)+ymin;
		/// fileout<<f+1<<":"<<x_value[f]<<" ";   
	 }
}

 void  normalize_y(vector<float> &a)
 {
  float max=*max_element(a.begin(),a.end()); 
   //<<endl;
 float min=*min_element(a.begin(),a.end());
 //  cout<<" min data = "<<min<<endl;
   //cout<<" differ  = "<<max-min<<endl;
// cout<<"max data = "<<max<<"            min data = "<<min<<"            differ  = "<<max-min<<endl;
// nomalize
 for(int f=0;f<a.size(); ++f)
	 {
		 a[f]=(1023-0)*(a[f]-min)/(max-min)+0 ;
		//features[f] = (ymax-ymin)*(features[f]-dMinValue)/(dMaxValue-dMinValue+1e-8)+ymin;
		// fileout<<f+1<<":"<<x_value[f]<<" ";   
	 }
 // compute  Bounding rectangle, it should be implemented in the main function !!!
 // the coordinates of the lower right corner is (x_max,y_min)
 // the coordinates of the upper left corner is (x_min,y_max)
}
 
 //xoy
 void creatpicture(vector<float> a,vector<float> b,vector<float> c,vector<int> color,int n,Mat img1,Mat img2)
{
	 for(int i=0;i<n;i++)
		{
			int row=static_cast<int>(floor(b[i]+0.5f));
			int col=static_cast<int>(floor(a[i]+0.5f));
			int differ=static_cast<int>(floor(c[i]+0.5f));
			//下面是方法一；利用的是指针的方法
			uchar*data1=img1.ptr<uchar>(row);  //     1126
			uchar*data2=img2.ptr<uchar>(row);  //     1126  作为一个缓冲矩阵来用
			data1[3*col]+=(color[i]-data2[3*col]);//b通道  时序的叠加
		   data1[3*col+1]+=(abs(differ-data2[3*col+1]));//g通道  缺少的坐标的差值
		    if(differ!=data2[3*col+1])  //如果当前坐标的颜色值不等于上一时刻进来的颜色值，则个数增加
			   ++data1[3*col+2];//r通道  相同坐标下不同时序的点的个数的叠加
		

			data2[3*col]=color[i];//b通道
			data2[3*col+1]=differ;//r通道
			
				//下面是方法二，采用的是.at 的方法，访问像素值
			//img1.at<Vec3b>(row,col)[0]+=(color[i]-img2.at<Vec3b>(row,col)[0]) ;//B通道，记录的是时序叠加
   //         img1.at<Vec3b>(row,col)[1]+=(abs(differ-img2.at<Vec3b>(row,col)[1]));//G通道，记录的是缺省坐标的做差叠加
			//if(color[i]!=img2.at<Vec3b>(row,col)[0])
			//	 ++img1.at<Vec3b>(row,col)[2];

			//img2.at<Vec3b>(row,col)[0]=color[i];
			//img2.at<Vec3b>(row,col)[1]=differ;



			//img.at<uint8_t>(row,col)+=c[i]/2; //color value stack when on the same coordinates 
		 }
	// imshow("2D point cloud picture",img1);
	  cout<<" creat 2D point cloud picture done!"<<endl;
 
}

//xoz
//void creatpicture_xoz(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,int n,Mat img1)
//{
//  vector <float> x_value;
//  vector <float> z_value;
//  vector <int> color;
// 
// for( size_t i=0;i<n;i++)
// {
//	 x_value.push_back( cloud->points[i].x+2);
//	 z_value.push_back( cloud->points[i].z+2);
//	 color.push_back(cloud->points[i].g);
//	//cout<<color[i]<<endl;
// }
//	 //nornalize 
//	 normalize(x_value); 
//	 normalize(z_value);
//	 for(int i=0;i<n;i++)
//		{
//			int row=(int)z_value[i];
//			int col=(int)x_value[i];
//			// pointer 
//			uchar *data=img1.ptr<uchar>(row);
//			data[col]+=color[i]/2;
//			//img1.at<uint16_t>(row,col)+=color[i]; //color value stack when on the same coordinates 
//		 }
////	 imshow("2D point cloud picture",img1);
//	 cout<<" creat 2D point cloud picture done!"<<endl;
// 
//}
//yoz
//void creatpicture_yoz(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,int n,Mat img1)
//{
//  vector <float> y_value;
//  vector <float> z_value;
//  vector <int> color;
// 
// for( size_t i=0;i<n;i++)
// {
//	 y_value.push_back( cloud->points[i].y+2);
//	 z_value.push_back( cloud->points[i].z+2);
//	 color.push_back(cloud->points[i].g);
//	//cout<<color[i]<<endl;
// }
//	 //nornalize 
//	 normalize(y_value); 
//	 normalize(z_value);
//	 for(int i=0;i<n;i++)
//		{
//			int row=(int)y_value[i];
//			int col=(int)z_value[i];
//			// pointer 
//			uchar *data=img1.ptr<uchar>(row);
//			data[col]+=color[i];
//			//img1.at<uint16_t>(row,col)+=color[i]; //color value stack when on the same coordinates 
//		 }
////	 imshow("2D point cloud picture",img1);
//	 cout<<" creat 2D point cloud picture done!"<<endl;
// 
//}




int main()
{
	//  1  read pcd file, use pointers to define,why not use another form to define?
	 CString pathin;
	 pathin = "F://MSR action 3D 实验//实验得到的数据//MHPC";        //diratory of the *.pcd file
	 CFileFind finder; //using this class of CFileFind to find the file in the diratory.
	BOOL working = finder.FindFile(pathin + "//*.pcd"); 
	int k=1;
	while (working) 
	{
		working = finder.FindNextFile();
		char path[100],title[100],title_out[100]; //to save the path variate
		sprintf(path, "%s", (LPCSTR)finder.GetFilePath());//修改属性，多字节字符集
		// 1  read source_cloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>); 
		pcl::PCDReader reader;
		reader.read<pcl::PointXYZRGB> (path, *source_cloud);                            //1025
		int num=source_cloud->points.size();   // get the size of source_cloud
		vector <float> x_value;
		vector <float> y_value;
        vector <float> z_value;
        vector <int> color;   
		for( size_t i=0;i<num;i++)
		 {
			 x_value.push_back( source_cloud->points[i].x+1);           //1123   +2   先省略
			 y_value.push_back( source_cloud->points[i].y+1);
			 z_value.push_back( source_cloud->points[i].z);
			 color.push_back(source_cloud->points[i].g);
			//cout<<color[i]<<endl;
		 }
		//cout<<k<<"  个MHPC的信息："<<endl;
		//2 normalize 
		 normalize_x_z(x_value); 
	     normalize_y(y_value);
		 normalize_x_z(z_value);
		  // 4 transform plane cloud that  were projected  from MHPC into two-dimensional image
		  //xoy 
		

		  //  3 得到灰度图片
		/*   Mat xoy(1024,512,CV_8UC3,Scalar(0)), temp_xoy(1024,512,CV_8UC3,Scalar(0));
		  creatpicture(x_value,y_value,z_value,color,num,xoy,temp_xoy);*/
		    Mat xoy(1024,512,CV_8UC3,Scalar(0)), temp_xoy(1024,512,CV_8UC3,Scalar(0));
		  creatpicture(x_value,y_value,z_value,color,num,xoy,temp_xoy);
	

		/*  Mat xoz(512,512,CV_8UC3,Scalar(0)),temp_xoz(512,512,CV_8UC3,Scalar(0));
		  creatpicture(x_value,z_value,y_value,color,num,xoz,temp_xoz);*/
		    Mat xoz(512,512,CV_8UC3,Scalar(0)),temp_xoz(512,512,CV_8UC3,Scalar(0));
		  creatpicture(x_value,z_value,y_value,color,num,xoz,temp_xoz);

		
		 /* Mat yoz(1024,512,CV_8UC3,Scalar(0)),temp_yoz(1024,512,CV_8UC3,Scalar(0));
		  creatpicture(z_value,y_value,x_value,color,num,yoz,temp_yoz);*/
		   Mat yoz(1024,512,CV_8UC3,Scalar(0)),temp_yoz(1024,512,CV_8UC3,Scalar(0));
		  creatpicture(z_value,y_value,x_value,color,num,yoz,temp_yoz);

		//  creatpicture_xoy(source_cloud,num,pointcloud_picture_xoy);
		  sprintf(title_out,"F://Action 3D//MSTD_Action_differ//%s_xoy.bmp",(LPCSTR)finder.GetFileTitle()); 
		  imwrite(title_out,xoy);
		   sprintf(title_out,"F://Action 3D//MSTD_Action_differ//%s_xoz.bmp",(LPCSTR)finder.GetFileTitle()); 
		  imwrite(title_out,xoz);
		   sprintf(title_out,"F://Action 3D//MSTD_Action_differ//%s_yoz.bmp",(LPCSTR)finder.GetFileTitle()); 
		  imwrite(title_out,yoz);
		//  cout<<title_out<<"have done!"<<endl;

		  //  Mat img1_shrink;
		  //resize(xoy, img1_shrink, Size(64, 128), (0, 0), (0, 0), 3);
		  //HOGDescriptor hog1(Size(64, 128), Size(16,16), Size(16, 16), Size(8, 8), 9);
		  //vector<float>descriptors1;
		  //int DescriptorDim1;
		  //hog1.compute(img1_shrink, descriptors1, Size(0, 0));
		  //cout << "描述子维数DescriptorDim1：" << DescriptorDim1 << endl;

		   k++;
		  cout<<"The "<<k<<" MHPC  have done!"<<endl;
		
	}

	return 0;
}


