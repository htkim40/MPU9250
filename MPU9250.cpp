#include <iostream> 
#include <unistd.h> //used to open/close device ports 
#include <fcntl.h>  //also used with device ports 
#include <sys/ioctl.h> 
#include <linux/i2c-dev.h>
 
#define GCONFIG  0b00001000

#define RBIT 0x80
#define MPUADDR 0x68 // if AD0 is set to 0, then 0x68, else if set to high then 0x69
#define WHOAMI 0x75

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F
#define XG_OFFSET_H 0x13
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C 
#define ACCEL_CONFIG_2 0x1D
#define LP_ACCEL_ODR 0x1E
#define WOM_THR 0x1F
#define FIFO_EN 0x23
#define I2C_MST_CTRL 0x24

class mpu9250{
	public: 
		unsigned char rbuffer[60], wbuffer[60] = {0}; 
		mpu9250();
		~mpu9250();
		void readWHOAMI();
		void readCONFIG();
		void writeCONFIG();
		void readGCONFIG();
		void writeGCONFIG(unsigned char gconfig);
	private: 
		char *filename = (char*)"/dev/i2c-1"; //device number may vary : "/dev/i2c-n"
		const int addr = 0x68; // the i2c adapter address 	
		int i2c_fd; //i2cdev file address 

		void writeBytes(unsigned char* data, int wlength);
		void readBytes(unsigned char regAddr, int rlength);
};

mpu9250::mpu9250(){
//--------Open the i2c bus----------------//
	if((i2c_fd = open(filename, O_RDWR))<0){
		std::cout<<"Failed to open the i2c bus"<<std::endl; 
		exit(0); //should update the exit codes to reflect error  
	}
}

mpu9250::~mpu9250(){
	close(i2c_fd);
}

void mpu9250::readBytes(unsigned char regAddr, int rlength){

	if(ioctl(i2c_fd, I2C_SLAVE, MPUADDR)<0){
		std::cout<<"Failed to aquire bus access and/or talk to i2c slave."<<std::endl; 
		exit(0);
	}

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
	if(ioctl(i2c_fd, I2C_SLAVE, MPUADDR)<0){
		std::cout<<"Failed to aquire bus access and/or talk to i2c slave."<<std::endl; 
		exit(0);
	}

	if(write(i2c_fd, data, wlength)!= wlength){
		std::cout<<"Failed to write to the i2c bus."<<std::endl; 
		return;
	}
}

void mpu9250::readWHOAMI(){
	readBytes(WHOAMI, 1);
}

void mpu9250::readCONFIG(){
	readBytes(CONFIG,1);
}


void mpu9250::writeCONFIG(){
/* Configuration takes in 1 byte. 
7: resereved 
6: fifo_mode -- 1) Additional writes will not be written to fifo when fifo is full
			  2) Additional writes will be written to the fifo, replacing the oldest data
5:3: EXT_SYNC_SET[2:0] external sync settings. //probably leave this as 0 
2:0: DLPF, not sure what DLPF is 
*/
}
void mpu9250::readGCONFIG(){
	readBytes(GYRO_CONFIG,1);
}

void mpu9250::writeGCONFIG(unsigned char gconfig){

/* Configuration takes in 1 byte. 
7:5) self-test 
4:3) gyro_frequence 00 = 250, 01 = 500, 10 = 1000, 11 = 2000
	dps 
   2) reserved 
1:0) fchoice bypass 

Generally speaking, we only want to modify values for bits 4:3.  */
wbuffer[0] = GYRO_CONFIG; 
wbuffer[1] = gconfig; 
writeBytes(wbuffer,2); 
}



int main(void)
{
mpu9250 mpu1; 
mpu1.readWHOAMI();
std::cout<<WHOAMI<<std::endl;
std::cout<<(int)mpu1.rbuffer[0]<<std::endl;
mpu1.readCONFIG();
std::cout<<(int)mpu1.rbuffer[0]<<std::endl;
mpu1.writeGCONFIG((unsigned char)GCONFIG);
mpu1.readGCONFIG();
std::cout<<(int)mpu1.rbuffer[0]<<std::endl;

return 0; 
}
