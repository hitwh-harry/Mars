
#include "LineDetect.h"

int main()
{
	// downSample();
	

	Mat srcImage;
	srcImage = imread("/home/harry/project/Mars/pictures/7_1.jpg"); //../pictures/4_1.jpg"  //768*1024

	if (srcImage.empty())
	{
		cout << "image not found" << endl;
		throw exception();
	}
	Mat img = srcImage.clone();

	LineDecect ld;
	vector<Point> p = ld.harrisCornorDetect(img, 3);
	ld.confirmLine(p, img, 10, 5);
	// ld.edgeDetect(img);
}