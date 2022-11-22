
#include "LineDetect.h"

int main()
{
	// downSample();
	

	Mat srcImage; //768*1024 rows*cols
	srcImage = imread("/home/harry/project/Mars/pictures/5_1.jpg"); //../pictures/4_1.jpg"  

	if (srcImage.empty())
	{
		cout << "image not found" << endl;
		throw exception();
	}
	Mat img = srcImage.clone();

	LineDecect ld(img);

	//边缘检测法
	// vector<Point> pv = ld.harrisCornorDetect(3);
	// ld.edgeFindLine(pv, 10, 5);
	// ld.edgeDetect();

	//垂线法
	vector<Point> pv = ld.harrisCornorDetect(20);
	ld.verticalFindLine(pv, 5);
	
}