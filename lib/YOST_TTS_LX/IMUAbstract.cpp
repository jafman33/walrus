#include "IMUAbstract.h"

namespace Cyberwing
{
	IMUAbstract::IMUAbstract(const IMUAbstract::MODE &mode,
		                       const uint32_t &receiveTimeout) :
	rawData_(),
	fusionedData_(),
	status_(IMUAbstract::STATUS::INIT),
	receiveTimeout_(receiveTimeout),
	tnm1Received_(0),
	dataCounter_(0),
	mode_(mode),
	quatOffset_{0.0F}
	{}

	IMUAbstract::~IMUAbstract()
	{}

	void IMUAbstract::resetTimeout(void)
	{
		tnm1Received_ = micros();
	}

	void IMUAbstract::setOffset(void)
	{
		float n0 = fusionedData_.quatRaw[0]*quatOffset_[0]
		- fusionedData_.quatRaw[1]*quatOffset_[1]
		- fusionedData_.quatRaw[2]*quatOffset_[2]
		- fusionedData_.quatRaw[3]*quatOffset_[3];

		float n1 = fusionedData_.quatRaw[0]*quatOffset_[1]
		+ fusionedData_.quatRaw[1]*quatOffset_[0]
		- fusionedData_.quatRaw[2]*quatOffset_[3]
		+ fusionedData_.quatRaw[3]*quatOffset_[2];

		float n2 = fusionedData_.quatRaw[0]*quatOffset_[2]
		+ fusionedData_.quatRaw[1]*quatOffset_[3]
		+ fusionedData_.quatRaw[2]*quatOffset_[0]
		- fusionedData_.quatRaw[3]*quatOffset_[1];

		float n3 = fusionedData_.quatRaw[0]*quatOffset_[3]
		- fusionedData_.quatRaw[1]*quatOffset_[2]
		+ fusionedData_.quatRaw[2]*quatOffset_[1]
		+ fusionedData_.quatRaw[3]*quatOffset_[0];

		fusionedData_.quat[0] = n0;
		fusionedData_.quat[1] = n1;
		fusionedData_.quat[2] = n2;
		fusionedData_.quat[3] = n3;
	}

	void IMUAbstract::tareCustom(void)
	{
		quatOffset_[0] = fusionedData_.quatRaw[0];
		quatOffset_[1] = -fusionedData_.quatRaw[1];
		quatOffset_[2] = -fusionedData_.quatRaw[2];
		quatOffset_[3] = -fusionedData_.quatRaw[3];
	}

	void IMUAbstract::computeEuler(void)
	{
		float w = fusionedData_.quat[0];
		float x = fusionedData_.quat[1];
		float y = fusionedData_.quat[2];
		float z = fusionedData_.quat[3];

		// roll (x-axis rotation)
		float sinr = +2.0 * (w * x + y * z);
		float cosr = +1.0 - 2.0 * (x * x + y * y);
		fusionedData_.euler[0] = atan2(sinr, cosr);

		// pitch (y-axis rotation)
		float sinp = +2.0 * (w * y - z * x);
		if (fabs(sinp) >= 1)
			fusionedData_.euler[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
		else
			fusionedData_.euler[1] = asin(sinp);

		// yaw (z-axis rotation)
		float siny = +2.0 * (w * z + x * y);
		float cosy = +1.0 - 2.0 * (y * y + z * z);
		fusionedData_.euler[2] = atan2(siny, cosy);

	}
}
