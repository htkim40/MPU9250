#include <iostream> 
#include <thread> 
#include <chrono>
#include <unistd.h> //used to open/close device ports 
#include <fcntl.h>  //also used with device ports 
#include <sys/ioctl.h> 
#include <linux/i2c-dev.h>
 

/*~~~~~~~~~~~~~Gyro definitions~~~~~~~~~~~~~~~
Gyro Full Scale Range, determines the sensitivity
GFS_SEL = 00, Ag = 131,  +/-250dps
GFS_SEL = 01, Ag = 65.5, +/-500dps
GFS_SEL = 10, Ag = 32.8, +/-1000dps
GFS_SEL = 11, Ag = 16.4, +/-5000dps
GFS_SEL should be asserted on bits [4:3] of GCONFIG1
then written into GYRO_CONFIG_CONFIG
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#define GFS_SEL 0b00
#define GCONFIG  0b00000000|(GFS_SEL<<3)
#define GYRO_CONFIG 0x1B
//define offsets to offload some of the extra calculations for bias correction 
#define XG_OFFSET_H 0x13
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48


/*~~~~~~~~~~~~~~Accel definitions~~~~~~~~~~~~~~
Accel Full Scale Range, determines the sensitivity
AFS_SEL = 00, Aa = 16.384,  +/-2g
AFS_SEL = 01, Aa = 8.192,    +/-4g
AFS_SEL = 10, Aa = 4.096,    +/-8g
AFS_SEL = 11, Aa = 2.048,    +/-16g
AFS_SEL should be asserted on bits [4:3] of ACONFIG1 
then written into ACCEL_CONFIG
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#define AFS_SEL 0b00
#define ACONFIG1  0b00000000|(AFS_SEL<<3) //accelerometer configuration 1
#define ACONFIG2 0b00000000 //accelerometer configuration2
#define ACCEL_CONFIG 0x1C //Register to write ACONFIG1 to
#define ACCEL_CONFIG_2 0x1D //Register 2 to write ACONFIG2 to
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

/*~~~~~~~~~~~Temperature Sensor~~~~~~~~~~~~~~
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42

/*~~~~~~~~~~~System Configurations~~~~~~~~~~~~*/
#define RBIT 0x80 	      //Read bit probably unecessary 
#define MPUADDR 0x68  // if AD0 is set to 0, then 0x68, else if set to high then 0x69-- MPU9250 address
#define WHOAMI 0x75     // Device ID. number. Reading should return 71 or 73

#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define LP_ACCEL_ODR 0x1E
#define WOM_THR 0x1F
#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C

/*~~~~~~~~~~~~~Self test registers~~~~~~~~~~~~~~~ 
self test registers are for end user to test manufacturing specs. 
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#define SELF_TEST_X_GYRO 0x00 
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F


#define FSAMPLE 10000 // in useconds -- this is 100hz

class mpu9250{
	public: 
		unsigned char rbuffer[60], wbuffer[60] = {0}; 
		double gyrX,gyrY,gyrZ,accX,accY,accZ;
		
		mpu9250();
		~mpu9250();

		void initializeMPU();
		void calibrateMPU();
		void updateSensors();
		void printRaw();
		void printSensors();

	private: 
		const char *filename = (char*)"/dev/i2c-1"; //device number may vary : "/dev/i2c-n"
		const int addr = 0x68; // the i2c adapter address 	
		int i2c_fd; //i2cdev file address 
		int16_t gyrXraw, gyrYraw, gyrZraw, accXraw, accYraw, accZraw; 
		double mgX, mgY, mgZ, maX, maY, maZ;
		double bgX, bgY, bgZ, baX, baY, baZ;

		void writeBytes(unsigned char* data, int wlength);
		void readBytes(unsigned char regAddr, int rlength);
};

mpu9250::mpu9250(){
//Open up the i2c bus, and save the port to i2c_fd 
	if((i2c_fd = open(filename, O_RDWR))<0){
		std::cout<<"Failed to open the i2c bus"<<std::endl; 
		exit(0); //should update the exit codes to reflect error  
	}
//set the i2c slave address to the MPU address 
	if(ioctl(i2c_fd, I2C_SLAVE, MPUADDR)<0){
		std::cout<<"Failed to aquire bus access and/or talk to i2c slave."<<std::endl; 
		exit(0);
	}
}

mpu9250::~mpu9250(){
//close the i2c port upon termination 
	close(i2c_fd);
}

void mpu9250::readBytes(unsigned char regAddr, int rlength){
	if(write(i2c_fd,&regAddr, 1)<1){
		std::cout<<"Failed to write to the i2c bus."<<std::endl; 
		return;
	}
	
	int count = read(i2c_fd, rbuffer, rlength);
	
	if(count<0){
		std::cout<<"Failed to read from the i2c bus"<<std::endl; 	
	} else if(count!= rlength) {
		std::cout<<"Short read, expected : "<<rlength<<", but got : "<<count<<std::endl; 
		return;
	}
}

void mpu9250::writeBytes(unsigned char* data, int wlength) {
	if(write(i2c_fd, data, wlength)!= wlength){
		std::cout<<"Failed to write to the i2c bus."<<std::endl; 
		return;
	}
}

void mpu9250::initializeMPU(){
	//Initialize MPU9250 by clearing sleep mode and tying 
	//clock to gyro PLL 
	wbuffer[0] = PWR_MGMT_1; 
	wbuffer[1] = 0x00; 
	writeBytes(wbuffer, 2);
	usleep(100); //delay 100ms 
	wbuffer[1] = 0x01; 
	writeBytes(wbuffer,2);

	//Write 0x03 turns off FSYNC, and sets the temperature
	//sensor, and gyro bandwidth to 42 and 41hz 
	//know that the trade off is that with the lower bandwidth 
	//there is also a delay in the sample of maximum 5.9ms for
	//gyros and then 4.8ms for the temperature sensors/accels 
	//Digital low pass filter can be bypassed by setting the in-
	//dividual sensor configurations:
	//GYRO_CONFIG, ACCEL_CONFIG
	wbuffer[0] = CONFIG; 
	wbuffer[1] = 0x03;
	writeBytes(wbuffer, 2);

	//Because of the 5.9ms delay, we may want to lower the 
	//sampling frequency to match the delay, but this loses 
	//data, so we will keep the sampling frequency at 1khz, 
	//and take the 6ms delay. which is almost unnoticable 
	//wbuffer[0] = SMPLRT_DIV;
	//wbuffer[1] = 0x04;

	//set the gyro full scale range 
	readBytes(GYRO_CONFIG,1);
	wbuffer[0] = GYRO_CONFIG; 
	wbuffer[1] = (rbuffer[0]&0x07);
	writeBytes(wbuffer,2); 
	// clears self-test bits and AFS bits [7:3]  0x07 = 0b00000111
	//keep the default full scale range [2:0] known to default to 00 

	//set the accel full scale range 
	readBytes(ACCEL_CONFIG,1); 
	wbuffer[0] = ACCEL_CONFIG;
	wbuffer[1] = (rbuffer[0]&0x07); 
	writeBytes(wbuffer,2);
	//Clears the self-test bits, and AFS bits
	//keep the default full scale range also known to be 00 

	//set the accelerometer sampling frequency by bypassing DLPF
	//This is only an option, but we want to keep the 1khz configur-
	//ration because of the 42hz LPF 
	//readBytes(ACCEL_CONFIG_2,1);
	//wbuffer[0] = ACCEL_CONFIG; 
	//wbuffer[1] = (rbuffer[0]&0xF0)|0x03;	
	//writeBytes(wbuffer,2);
	 
}

void mpu9250::updateSensors(){
	gyrXraw = gyrYraw = gyrZraw = accXraw = accYraw = accZraw =0;
	readBytes(GYRO_XOUT_H,1); 
	gyrXraw |= rbuffer[0]<<8; 
	readBytes(GYRO_XOUT_L,1); 
	gyrXraw |= rbuffer[0]; 
	gyrX = (double)gyrXraw/131; 

	readBytes(GYRO_YOUT_H,1); 
	gyrYraw |= rbuffer[0]<<8; 
	readBytes(GYRO_YOUT_L,1); 
	gyrYraw |= rbuffer[0]; 
	gyrY= (double)gyrYraw/131;

	readBytes(GYRO_ZOUT_H,1); 
	gyrZraw |= rbuffer[0]<<8; 
	readBytes(GYRO_ZOUT_L,1); 
	gyrZraw |= rbuffer[0]; 
	gyrZ = (double)gyrZraw/131; 	
	
	readBytes(ACCEL_XOUT_H,1);
	accXraw|=rbuffer[0]<<8;
	readBytes(ACCEL_XOUT_L,1); 
	accXraw|=rbuffer[0];
	accX = (double)accXraw/16384;

	readBytes(ACCEL_YOUT_H,1);
	accYraw|=rbuffer[0]<<8;
	readBytes(ACCEL_YOUT_L,1); 
	accYraw|=rbuffer[0];
	accY = (double)accYraw/16384;

	readBytes(ACCEL_ZOUT_H,1);
	accZraw|=rbuffer[0]<<8;
	readBytes(ACCEL_ZOUT_L,1); 
	accZraw|=rbuffer[0];
	accZ = (double)accZraw/16384;
}

void mpu9250::printRaw(){
	std::cout<<"raw gyro  : <"<<gyrXraw<<","<<gyrYraw<<","<<gyrZraw<<">"<<std::endl;
	std::cout<<"raw accel : <"<<accXraw<<","<<accYraw<<","<<accZraw<<">"<<std::endl;
}

void mpu9250::printSensors(){
	std::cout<<"gyro  : <"<<gyrX<<","<<gyrY<<","<<gyrZ<<">"<<std::endl;
	std::cout<<"accel : <"<<accX<<","<<accY<<","<<accZ<<">"<<std::endl;
}
/*
void mpu9250::calibrateMPU()
{


}
*/

void fsynchro(){
	
	auto begin = std::chrono::high_resolution_clock::now() ;
	while(1){
	auto end = std::chrono::high_resolution_clock::now();
	auto tet = std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count();
//	std::cout<<tet<<std::endl;
	if(tet>FSAMPLE)break;
	}
}

int main(void)
{
	mpu9250 mpu1; 
	mpu1.initializeMPU();
	auto begin = std::chrono::high_resolution_clock::now(); 
	

for(int i = 0;i<1000;i++){
	std::thread synchro(fsynchro);
	mpu1.updateSensors();
	synchro.join();
}
	auto end = std::chrono::high_resolution_clock::now();
	auto tet = std::chrono::duration_cast<std::chrono::microseconds>(end-begin).count();
	std::cout<<tet<<std::endl;

//std::thread iterate_th(mpu1.updateMPU)
//std::thread timer_th(fsync)

return 0; 
}
