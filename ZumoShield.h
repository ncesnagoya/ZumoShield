#include <ZumoBuzzer.h>
#include <ZumoMotors.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <avr/pgmspace.h>
#include <LSM303.h>
#include <L3G.h>
#include <RunningAverage.h>
#include <Accelerometer.h>

template <typename T> float heading(LSM303::vector<T> v, LSM303 *compass)
{
	float x_scaled =  2.0*(float)(v.x - compass->m_min.x) / (compass->m_max.x - compass->m_min.x) - 1.0;
	float y_scaled =  2.0*(float)(v.y - compass->m_min.y) / (compass->m_max.y - compass->m_min.y) - 1.0;

	float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
	if (angle < 0)
	  angle += 360;
	return angle;
}

#define CRB_REG_M_2_5GAUSS 0x60 // CRB_REG_M value for magnetometer +/-2.5 gauss full scale
#define CRA_REG_M_220HZ    0x1C // CRA_REG_M value for magnetometer 220 Hz update rate

class ZumoCompass : public LSM303
{
  public:
	ZumoCompass() {};
	void begin() {
		Wire.begin();
		// Initiate LSM303
		init();
		// Enables accelerometer and magnetometer
		enableDefault();
		writeReg(LSM303::CRB_REG_M, CRB_REG_M_2_5GAUSS); // +/- 2.5 gauss sensitivity to hopefully avoid overflow problems
		writeReg(LSM303::CRA_REG_M, CRA_REG_M_220HZ);    // 220 Hz compass update rate  
  
		// Set calibrated values to compass.m_max and compass.m_min
		m_max.x = -447;
		m_max.y = -673;
		m_min.x = -2252;
		m_min.y = -2511; 
	};
	
	float averageHeading() {
		LSM303::vector<int32_t> avg = {0, 0, 0};

		for(int i = 0; i < 10; i ++)
		  {
			  read();
			  avg.x += m.x;
			  avg.y += m.y;
		  }
		avg.x /= 10.0;
		avg.y /= 10.0;
		
		// avg is the average measure of the magnetic vector.
		return ::heading(avg, (LSM303 *)this);
	}
};

class ZumoGyro : public L3G
{
  public:
	ZumoGyro(void){};

/* turnAngle is a 32-bit unsigned integer representing the amount
the robot has turned since the last time turnSensorReset was
called. This is computed solely using the Z axis of the gyro, so
it could be inaccurate if the robot is rotated about the X or Y
axes.

Our convention is that a value of 0x20000000 represents a 45
degree counter-clockwise rotation.  This means that a uint32_t
can represent any angle between 0 degrees and 360 degrees.  If
you cast it to a signed 32-bit integer by writing
(int32_t)turnAngle, that integer can represent any angle between
-180 degrees and 180 degrees. */
	uint32_t turnAngle = 0;

	int32_t turnAngleDegree = 0;

// turnRate is the current angular rate of the gyro, in units of
// 0.07 degrees per second.
	int16_t turnRate;

// This is the average reading obtained from the gyro's Z axis
// during calibration.
	int16_t gyroOffset;

// This variable helps us keep track of how much time has passed
// between readings of the gyro.
	uint16_t gyroLastUpdate = 0;

	// This constant represents a turn of 45 degrees.
	const int32_t turnAngle45 = 0x20000000;

	// This constant represents a turn of 90 degrees.
	const int32_t turnAngle90 = turnAngle45 * 2;

	// This constant represents a turn of approximately 1 degree.
	const int32_t turnAngle1 = (turnAngle45 + 22) / 45;

	// This should be called to set the starting point for measuring
	// a turn.  After calling this, turnAngle will be 0.
	void turnSensorReset() {
		gyroLastUpdate = micros();
		turnAngle = 0;
		turnAngleDegree = 0;
	  }

	// Read the gyro and update the angle.  This should be called as
	// frequently as possible while using the gyro to do turns.
	void turnSensorUpdate() {
		// Read the measurements from the gyro.
		read();
		turnRate = g.z - gyroOffset;
		
		// Figure out how much time has passed since the last update (dt)
		uint16_t m = micros();
		uint16_t dt = m - gyroLastUpdate;
		gyroLastUpdate = m;
		
		// Multiply dt by turnRate in order to get an estimation of how
		// much the robot has turned since the last update.
		// (angular change = angular velocity * time)
		int32_t d = (int32_t)turnRate * dt;
		  
		// The units of d are gyro digits times microseconds.  We need
		// to convert those to the units of turnAngle, where 2^29 units
		// represents 45 degrees.  The conversion from gyro digits to
		// degrees per second (dps) is determined by the sensitivity of
		// the gyro: 0.07 degrees per second per digit.
		//
		// (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
		// = 14680064/17578125 unit/(digit*us)
		turnAngle += (int64_t)d * 14680064 / 17578125;
		turnAngleDegree = ((int32_t)turnAngle / turnAngle1);
	}

	void turnSensorSetup() {
		Wire.begin();
		init();

		// 800 Hz output data rate,
		// low-pass filter cutoff 100 Hz
		writeReg(L3G::CTRL1, 0b11111111);

		// 2000 dps full scale
		writeReg(L3G::CTRL4, 0b00100000);

		// High-pass filter disabled
		writeReg(L3G::CTRL5, 0b00000000);

		// Delay to give the user time to remove their finger.
		delay(500);

		// Calibrate the gyro.
		int32_t total = 0;
		for (uint16_t i = 0; i < 1024; i++)
		  {
			  // Wait for new data to be available, then read it.
			  while(!readReg(L3G::STATUS_REG) & 0x08);
			  read();
			  // Add the Z axis reading to the total.
			  total += g.z;
		  }
		gyroOffset = total / 1024;

		// Display the angle (in degrees from -180 to 180) until the
		// user presses A.
		turnSensorReset();   
	};
};

class ZumoBuzzer2 : public ZumoBuzzer
{
  public:
	ZumoBuzzer2(){};
	void playOn(void) {play(">g32>>c32");};
	void playStart(void) {playNote(NOTE_G(4), 500, 15);}
	void playNum(int cnt) {
		for (int i = 0; i < cnt; i++){
			delay(1000);
			playNote(NOTE_G(3), 50, 12);
		}  
	};
};

class ZumoLED
{
  public:
	ZumoLED(){pinMode(13, OUTPUT);};
	void on(){digitalWrite(13, HIGH);};
	void off(){digitalWrite(13, LOW);};
	void set(int i){
		digitalWrite(13, (i == 1)? HIGH : LOW);
	};
};

class ZumoReflectanceSensorArray2 : public ZumoReflectanceSensorArray {
  public:
	unsigned int values[6];
	ZumoReflectanceSensorArray2() : ZumoReflectanceSensorArray(QTR_NO_EMITTER_PIN){
		
	};
	void update(void){
		read(values);
	}
	unsigned int value(int i){
		if((i <= 6) && (i > 0)) {
			return values[i-1];
		}
		return 0;
	}
};

ZumoReflectanceSensorArray2 reflectances;
Pushbutton  button(ZUMO_BUTTON);
ZumoLED     led;
ZumoBuzzer2 buzzer;
ZumoMotors  motors;
ZumoCompass compass;
ZumoGyro    gyro;
