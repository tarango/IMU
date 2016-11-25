

#include "km.h"
/* Kalman filter variables */
	double Q_angleX=0.00000001; // Process noise variance for the accelerometer
	double Q_biasX=0.00000001; // Process noise variance for the gyro bias
	double R_measureX=0.00000001; // Measurement noise variance - this is actually the variance of the measurement noise

	double Q_angleY=0.00000000000000000001; // Process noise variance for the accelerometer
	double Q_biasY=0.0000000000000000000001; // Process noise variance for the gyro bias
	double R_measureY=0.00000000000000000001; // Measurement noise variance - this is actually the variance of the measurement noise

	double Q_angleZ=5000;//0.009; // Process noise variance for the accelerometer
	double Q_biasZ=900000;//0.003; // Process noise variance for the gyro bias
	double R_measureZ=250000;//0.03; // Measurement noise variance - this is actually the variance of the measurement noise

	double Q_angle=0.001; // Process noise variance for the accelerometer
	double Q_bias=0.003; // Process noise variance for the gyro bias
	double R_measure=0.03; // Measurement noise variance - this is actually the variance of the measurement noise

	double angleX=0; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	double angleY=0; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	double angleZ=0; // The angle calculated by the Kalman filter - part of the 2x1 state vector
	//double angle;//temporary
	double biasX=0; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	double biasY=0; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
	double biasZ=0; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector

	double rateX=0; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
	double rateY=0; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
	double rateZ=0; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

	double rate=0;

	double PX[2][2]=
	{
		{0, 0},
		{0, 0}// Error covariance matrix - This is a 2x2 matrix
	};
	double PY[2][2]=
	{
		{0, 0},
		{0, 0}// Error covariance matrix - This is a 2x2 matrix
	};
	double PZ[2][2]=
	{
		{0, 0},
		{0, 0}// Error covariance matrix - This is a 2x2 matrix
	};
	double KX[2]={0,0}; // Kalman gain - This is a 2x1 vector
	double KY[2]={0,0}; // Kalman gain - This is a 2x1 vector
	double KZ[2]={0,0}; // Kalman gain - This is a 2x1 vector

	double yX=0; // Angle difference
	double yY=0; // Angle difference
	double yZ=0; // Angle difference

	double SX=0; // Estimate error
	double SY=0; // Estimate error
	double SZ=0; // Estimate error

	double getAngleX(double newAngle, double newRate, double dt) {
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
		// Modified by Kristian Lauszus
		// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

		// Discrete Kalman filter time update equations - Time Update ("Predict")
		// Update xhat - Project the state ahead
		/* Step 1 */
		rateX = newRate - biasX;
		angleX += dt * rateX;

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		PX[0][0] += dt * (dt*PX[1][1] - PX[0][1] - PX[1][0] + Q_angleX);
		PX[0][1] -= dt * PX[1][1];
		PX[1][0] -= dt * PX[1][1];
		PX[1][1] += Q_biasX * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		SX = PX[0][0] + R_measureX;
		/* Step 5 */
		KX[0] = PX[0][0] / SX;
		KX[1] = PX[1][0] / SX;

		// Calculate angle and bias - Update estimate with measurement zk (newAngle)
		/* Step 3 */
		yX = newAngle - angleX;
		/* Step 6 */
		angleX += KX[0] * yX;
		biasX += KX[1] * yX;

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		PX[0][0] -= KX[0] * PX[0][0];
		PX[0][1] -= KX[0] * PX[0][1];
		PX[1][0] -= KX[1] * PX[0][0];
		PX[1][1] -= KX[1] * PX[0][1];

		return angleX;
	}
	double getAngleY(double newAngle, double newRate, double dt) {
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
		// Modified by Kristian Lauszus
		// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

		// Discrete Kalman filter time update equations - Time Update ("Predict")
		// Update xhat - Project the state ahead
		/* Step 1 */
		rateY = newRate - biasY;
		angleY += dt * rateY;

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		PY[0][0] += dt * (dt*PY[1][1] - PY[0][1] - PY[1][0] + Q_angleY);
		PY[0][1] -= dt * PY[1][1];
		PY[1][0] -= dt * PY[1][1];
		PY[1][1] += Q_biasY * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		SY = PY[0][0] + R_measureY;
		/* Step 5 */
		KY[0] = PY[0][0] / SY;
		KY[1] = PY[1][0] / SY;

		// Calculate angle and bias - Update estimate with measurement zk (newAngle)
		/* Step 3 */
		yY = newAngle - angleY;
		/* Step 6 */
		angleY += KY[0] * yY;
		biasY += KY[1] * yY;

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		PY[0][0] -= KY[0] * PY[0][0];
		PY[0][1] -= KY[0] * PY[0][1];
		PY[1][0] -= KY[1] * PY[0][0];
		PY[1][1] -= KY[1] * PY[0][1];

		return angleY;
		}
	double getAngleZ(double newAngle, double newRate, double dt) {
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
		// Modified by Kristian Lauszus
		// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

		// Discrete Kalman filter time update equations - Time Update ("Predict")
		// Update xhat - Project the state ahead
		/* Step 1 */
		rateZ = newRate - biasZ;
		angleZ += dt * rateZ;

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		PZ[0][0] += dt * (dt*PZ[1][1] - PZ[0][1] - PZ[1][0] + Q_angleZ);
		PZ[0][1] -= dt * PZ[1][1];
		PZ[1][0] -= dt * PZ[1][1];
		PZ[1][1] += Q_biasZ * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		SZ = PZ[0][0] + R_measureZ;
		/* Step 5 */
		KZ[0] = PZ[0][0] / SZ;
		KZ[1] = PZ[1][0] / SZ;

		// Calculate angle and bias - Update estimate with measurement zk (newAngle)
		/* Step 3 */
		yZ = newAngle - angleZ;
		/* Step 6 */
		angleZ += KZ[0] * yZ;
		biasZ += KZ[1] * yZ;

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		PZ[0][0] -= KZ[0] * PZ[0][0];
		PZ[0][1] -= KZ[0] * PZ[0][1];
		PZ[1][0] -= KZ[1] * PZ[0][0];
		PZ[1][1] -= KZ[1] * PZ[0][1];

		return angleZ;
		}
	 void setAngleX(double newAngle) { angleX = newAngle; }; // Used to set angle, this should be set as the starting angle for X
	 void setAngleY(double newAngle) { angleY = newAngle; }; // Used to set angle, this should be set as the starting angle for Y
	 void setAngleZ(double newAngle) { angleZ = newAngle; }; // Used to set angle, this should be set as the starting angle for Y
	 double getRate() { return rate; }; // Return the unbiased rate

	 //These are used to tune the Kalman filter
	 void setQangle(double newQ_angle) { Q_angle = newQ_angle; };
	 void setQbias(double newQ_bias) { Q_bias = newQ_bias; };
	 void setRmeasure(double newR_measure) { R_measure = newR_measure; };

	 double getQangle() { return Q_angle; };
	 double getQbias() { return Q_bias; };
	 double getRmeasure() { return R_measure; };
