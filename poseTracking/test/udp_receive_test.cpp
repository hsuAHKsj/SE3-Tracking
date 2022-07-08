#include "udp_reciever.h"

int main()
{
	imuDataPack iData;
	imu_com imu;
	while (imu.get_imu_data(iData))
	{
		printf("Message form server: %lf  %lf  %lf\n", iData.a[0], iData.a[1], iData.a[2]);

		Sleep(10);
	}
	return 0;
}