import processing.serial.*;

class Hardware {
	Serial arduino;

	Hardware( PApplet parent ) {
	  arduino = new Serial(parent, "/dev/tty.usbmodem431",9600);
	}	

	/*
	 Control byte: 1MMDHCCC
	 Value byte:   0PPPPPPP
	 M = Which motor
	 D = Motor direction
	 H = High bit of power
	 C = Checksum
	 P = Power value
	 
	 Checksum = number of ones in value byte
	 Power = H*127 + value byte
	 */
	void sendCommand( int motor, int direction, int power ) {
	  int control = 0x80;
	  control |= (motor & 0x3) << 5;
	  control |= direction * 0x10;
	  control |= (power > 127 ? 0x8 : 0);
	  
	  power &= 0x7F;
	  
	  int bits = 0;
	  for (int i=0; i<7; i++)
	     bits += (power>>i) & 0x1;
	  control |= bits;

	  arduino.write(control);
	  arduino.write(power);
	}

	void error() {
		arduino.write(0);
	}
}
