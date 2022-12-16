
#include "LineDetect.h"
#include "Util.h"
#include "Locate.h"

int main()
{
	// Util::calibrate();
	// Util::downSample();

	Mat srcImage; // rows*cols
	srcImage = imread("/home/harry/project/Mars/pictures/2_1.jpg"); //../pictures/4_1.jpg"  

	if (srcImage.empty())
	{
		cout << "image not found" << endl;
		throw exception();
	}
	Mat img = srcImage.clone();
	// cout<<img.rows<<img.cols<<endl;
	
    // circle(img, Point(243, 233), 2, Scalar(0, 0, 255), 1); //左上 (-66,-66.5,0)
	// circle(img, Point(244, 314), 2, Scalar(0, 0, 255), 1); //左下 (-66,66.5,0)
    // circle(img, Point(325, 311), 2, Scalar(0, 0, 255), 1); //右下 (66,66.5,0)
	// circle(img, Point(323, 231), 2, Scalar(0, 0, 255), 1); //中左上 (66,-66.5,0)
	// namedWindow("img", 0);
    // imshow("img", img);
    // waitKey(0);

	Mat templ=imread("../pictures/template/1.jpg");
	// Mat templ=img(Range(203,341),Range(213,355));
	// namedWindow("img", 0);
    // imshow("img", templ);
    // waitKey(0);
	// imwrite("../pictures/template/1.jpg",templ);
	
	LineDecect ld(img);
	ld.templateMatch(templ);
	ld.houghDetect();
	
	
	// // // 边缘检测法
	// vector<Point> pv;
	// ld.harrisCornorDetect(pv, 5);
	// ld.edgeFindLine(pv, 10, 5);
	// ld.edgeDetect();

	//垂线法
	// ld.harrisCornorDetect(pv, 20);
	// ld.verticalFindLine(pv, 10, 10);

	// Locate locate;
	// locate.pnp();
	// locate.computeAngle();
}