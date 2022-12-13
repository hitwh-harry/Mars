
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
	cout<<img.rows<<img.cols<<endl;
	
    circle(img, Point(254, 412), 2, Scalar(0, 0, 255), 1); //左上 (-66,-66.5,0)
	circle(img, Point(257, 254), 2, Scalar(0, 0, 255), 1); //左下 (-66,66.5,0)
    circle(img, Point(405, 255), 2, Scalar(0, 0, 255), 1); //右下 (66,66.5,0)
	circle(img, Point(417, 409), 2, Scalar(0, 0, 255), 1); //中左上 (66,-66.5,0)
	namedWindow("img", 0);
    imshow("img", img);
    waitKey(0);

	// LineDecect ld(img);

	// vector<Point> pv;
	
	// // 边缘检测法
	// ld.harrisCornorDetect(pv, 10);
	// ld.edgeFindLine(pv, 10, 5);
	// ld.edgeDetect();

	//垂线法
	// ld.harrisCornorDetect(pv, 20);
	// ld.verticalFindLine(pv, 10, 10);

	// Locate locate;
	// locate.pnp();
	// locate.computeAngel();
}