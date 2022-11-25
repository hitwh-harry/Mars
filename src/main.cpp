
#include "LineDetect.h"

int main()
{
	// downSample();
	

	Mat srcImage; //768*1024 rows*cols
	srcImage = imread("/home/harry/project/Mars/pictures/8_1.jpg"); //../pictures/4_1.jpg"  

	if (srcImage.empty())
	{
		cout << "image not found" << endl;
		throw exception();
	}
	Mat img = srcImage.clone();

	LineDecect ld(img);

	vector<Point> pv;
	
	//边缘检测法
	// ld.harrisCornorDetect(pv, 3);
	// ld.edgeFindLine(pv, 10, 5);
	// ld.edgeDetect();

	//垂线法
	ld.harrisCornorDetect(pv, 20);
	ld.verticalFindLine(pv, 10, 10);
	
}