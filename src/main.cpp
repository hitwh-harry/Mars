
#include "LineDetect.h"
#include "Calibration.h"
#include "Locate.h"

int main()
{
	// downSample();
	// Calibration::calibrate();

	// Mat srcImage; // rows*cols
	// srcImage = imread("/home/harry/project/Mars/pictures/1.jpg"); //../pictures/4_1.jpg"  

	// if (srcImage.empty())
	// {
	// 	cout << "image not found" << endl;
	// 	throw exception();
	// }
	// Mat img = srcImage.clone();

    // circle(img, Point(2120, 885), 10, Scalar(0, 0, 255), 3); //左上 (-64,64.5,0)
	// circle(img, Point(2077, 1890), 10, Scalar(0, 0, 255), 3); //左下 (-64,-64.5,0)
    // circle(img, Point(3107, 2042), 10, Scalar(0, 0, 255), 3); //右下 (64,-64.5,0)
	// circle(img, Point(2262, 1056), 10, Scalar(0, 0, 255), 3); //中左上 (-44.4,44.5,0)
	// namedWindow("harris corner", 0);
    // imshow("harris corner", img);
    // waitKey(0);

	// LineDecect ld(img);

	// vector<Point> pv;
	
	// 边缘检测法
	// ld.harrisCornorDetect(pv, 10);
	// ld.edgeFindLine(pv, 10, 5);
	// ld.edgeDetect();

	//垂线法
	// ld.harrisCornorDetect(pv, 20);
	// ld.verticalFindLine(pv, 10, 10);

	Locate locate;
	locate.pnp();
	locate.computeAngel();
}