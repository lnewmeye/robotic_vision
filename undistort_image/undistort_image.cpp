#include <iostream>
#include <opencv2/core.hpp>

using std::cout;
using std::endl;

// Define intrinsic and extrinsic parameters
#define INTRINSIC_PARAMS  [25.62525453049683, 0, 319.9983458278912; \\
                            0, 86.3109544090008, 239.9998104739093; \\
                            0, 0, 1]
#define DISTORTION_PARAMS [-0.01175394258213657;   \\
                            3.389305344966451e-05; \\
                            0.001906432852957746;  \\
                           -0.003736862025957332;  \\
                           -2.257617272719852e-08]

// Define image parameters
#define IMAGE_SEQUENCE {"Close", "Far", "Turn"}
#define IMAGE_EXTENSION ".jpg"
#define IMAGE_FOLDER "images/"
#define IMAGE_QUANTITY 3

int main()
{
	cout << "Hello World!" << endl;
	return 0;
}
