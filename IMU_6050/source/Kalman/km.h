
#ifndef KM_H_
#define KM_H_

double getAngleX(double newAngle, double newRate, double dt);
double getAngleY(double newAngle, double newRate, double dt);
double getAngleZ(double newAngle, double newRate, double dt);

void setAngleX(double newAngle);
void setAngleY(double newAngle); // Used to set angle, this should be set as the starting angle
void setAngleZ(double newAngle);
double getRate(); // Return the unbiased rate

//These are used to tune the Kalman filter
void setQangle(double newQ_angle);
void setQbias(double newQ_bias);
void setRmeasure(double newR_measure);

double getQangle() ;
double getQbias() ;
double getRmeasure();


#endif
