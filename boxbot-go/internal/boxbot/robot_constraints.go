package boxbot

const (
	MaxVelocityArm   float64 = 15.0                                            // coord/s
	MaxAccelArm      float64 = 7.0                                             // coord/s^2
	AccelTimeArm             = MaxVelocityArm / MaxAccelArm                    // s
	AccelDistArm             = 0.5 * MaxAccelArm * AccelTimeArm * AccelTimeArm // coord
	CreepVelocityArm float64 = 0.5                                             // coord

	MaxVelocityBase   float64 = 1.2                                                // rad/s
	MaxAccelBase      float64 = 0.5                                                // rad/s^2
	AccelTimeBase             = MaxVelocityBase / MaxAccelBase                     // s
	AccelDistBase             = 0.5 * MaxAccelBase * AccelTimeBase * AccelTimeBase // rad
	CreepVelocityBase float64 = 0.1                                                // rad

	MaxVelocityServo   float64 = 15.0                                                  // degree/s
	MaxAccelServo      float64 = 5.0                                                   // degree/s^2
	AccelTimeServo             = MaxVelocityServo / MaxAccelServo                      // s
	AccelDistServo             = 0.5 * MaxAccelServo * AccelTimeServo * AccelTimeServo // degree
	CreepVelocityServo float64 = 0.5                                                   // degree

	MaxVelocityTower   float64 = 10.0                                                  // coord/s
	MaxAccelTower      float64 = 4.0                                                   // coord/s^2
	AccelTimeTower             = MaxVelocityTower / MaxAccelTower                      // s
	AccelDistTower             = 0.5 * MaxAccelTower * AccelTimeTower * AccelTimeTower // coord
	CreepVelocityTower float64 = 0.4                                                   // coord
)
