
#include <iostream>
#include <bitset>

#include "Wire.h"

using namespace std;
	
#define TARGET_ADDR 32	//0x20 is standard PCA9555 address
#define TARGET_REG 0
#define NUM_REGS 2		//PCA9555 has 8 registers

void printRegisters()
{
	uint8_t ret = 0;
	
	Wire.beginTransmission(TARGET_ADDR);
	Wire.write(0); //start reading at first register
	Wire.endTransmission();
	Wire.requestFrom(TARGET_REG, NUM_REGS);
	
	for(int i=0; i<NUM_REGS;i++)
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