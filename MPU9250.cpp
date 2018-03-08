#include <iostream> 
#include <unistd.h> //used to open/close device ports 
#include <fcntl.h>  //also used with device ports 
#include <sys/ioctl.h> 
#include <linux/i2c-dev.h>
 
#define RBIT 0x80
#define MPUADDR 0x68 // if AD0 is set to 0, then 0x68, else if set to high then 0x69
//#define RMPUADDR 0xE8
#define WHOAMI 0x75

class mpu9250{
	public: 
		unsigned char rbuffer[60], wbuffer[60] = {0}; 
		mpu9250();
		~mpu9250();
		void readWHOAMI();
		void readBytes(int regAddr, int rlength);
	private: 
		char *filename = (char*)"/dev/i2c-1"; //device number may vary : "/dev/i2c-n"
		const int addr = 0x68; // the i2c adapter address 	
		int i2c_fd; //i2cdev file address 

		void writeMPU(int wlength); 

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

void mpu9250::readBytes(int regAddr, int rlength){

	if(ioctl(i2c_fd, I2C_SLAVE, MPUADDR)<0){
		std::cout<<"Failed to aquire bus access and/or talk to i2c slave."<<std::endl; 
		exit(0);
	}
	wbuffer[0] = regAddr;
	if(write(i2c_fd, wbuffer, 1)<1){
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

void mpu9250::writeMPU(int wlength) {
	
	if(ioctl(i2c_fd, I2C_SLAVE,MPUADDR)<0){
		std::cout<<"Failed to aquire bus access and/or talk to i2c slave."<<std::endl; 
		exit(0);
	}

	if(write(i2c_fd, wbuffer, wlength)!= wlength){
		std::cout<<"Failed to write to the i2c bus."<<std::endl; 
		return;
	}
}

void mpu9250::readWHOAMI(){
	
	if(ioctl(i2c_fd, I2C_SLAVE, MPUADDR)<0){
		std::cout<<"Failed to aquire bus access and/or talk to i2c slave."<<std::endl; 
		exit(0);
	}
	wbuffer[0] = WHOAMI;
	if(write(i2c_fd, wbuffer, 1)<1){
		std::cout<<"Failed to write to the i2c bus."<<std::endl; 
		return;
	}
/*
	if(write(i2c_fd, wbuffer, 0)!= 0){
		std::cout<<"Failed to write 'read' to the i2c bus."<<std::endl; 
		return;
	}*/
//	readMPU(1);
}




int main(void)
{
mpu9250 mpu1; 
mpu1.readWHOAMI();
std::cout<<WHOAMI<<std::endl;
std::cout<<(int)mpu1.rbuffer[0]<<std::endl;
mpu1.readBytes(0x3B, 2);
std::cout<<(int)mpu1.rbuffer[1]<< mpu1.rbuffer[2]<<std::endl;


return 0; 
}
