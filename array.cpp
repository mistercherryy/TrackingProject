
//auto a = 2.4, b = 4.22, c = 2.5, d = 3.6, e = 5.003;

//what i want to do
//std::vector<double> arr[5] = { a, b, c, d, e };

/* trackerdan elde edilen matris cinsi;
struct HmdMatrix34_t
{
	float m[3][4];
};
*/

//data type rtde takes
// std::vector<double> joint_q = { -1.54, -1.83, -2.28, -0.59, 1.60, 0.023 };
// First argument is the pose 6d vector followed by speed and acceleration
	//rtde_control.moveL({ -0.143, -0.435, 0.20, -0.001, 3.12, 0.04 }, 0.2, 0.2);

/* -----------yapilacaklar
-önce dereceleri radyan cinsine çevir----->(zaten radian)
-sonra bunlari vektör içine at
-en son robotla bunu paylas
*/