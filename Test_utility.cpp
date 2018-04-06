
#include <iostream>
#include <bitset>

#include "Wire.h"

using namespace std;
	
#define TARGET_ADDR 32	//0x20 is standard PCA9555 address
#define TARGET_REG 0
#define NUM_REGS 7//PCA9555 has 8 registers

void printRegisters()
{
	uint8_t ret = 0;
	
	Wire.beginTransmission(TARGET_ADDR);
	Wire.write(6); //write to port 0 config register
	Wire.write(0xAA); //set bit mask 10101010
	Wire.endTransmission();

//	Wire.beginTransmission(TARGET_ADDR);
//	Wire.write(0);
//	Wire.endTransmission();
	Wire.requestFrom(TARGET_ADDR, NUM_REGS,TARGET_REG,0,0); //read the state of all registers
	
	for(int i=TARGET_REG; i<8;i++)
	{
		ret = Wire.read();
		cout<<"Register "<<i<<": "<<bitset<8>(ret)<<endl;
	}

	
}

int main (int argc, char *argv[]) 
{
	cout<<"Programm to test Wire library functionality"<<endl;
	
	Wire.begin();
	printRegisters();
	Wire.end();
	cout<<"Done"<<endl;
	return 0;
}
