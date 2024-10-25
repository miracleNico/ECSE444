#include <stdint.h>
#include <math.h>
#define NUM_SAMPLES 200

uint8_t triwave_125hz(int counter){
	switch(counter%8){
		case 0: return 127;
		case 1: return 191;
		case 2: return 255;
		case 3: return 191;
		case 4: return 127;
		case 5: return 63;
		case 6: return 0;
		case 7: return 63;
	}
}

uint8_t sawwave_100hz(int counter){
	switch(counter%10){
		case 0: return 127;
		case 1: return 159;
		case 2: return 191;
		case 3: return 223;
		case 4: return 255;
		case 5: return 127;
		case 6: return 0;
		case 7: return 32;
		case 8: return 64;
		case 9: return 96;
	}
}

uint32_t* getSineWave(int freq) {
    int size = 40000/freq;
    uint8_t sineWave[size];
    double step = (2.0 * M_PI) / size;

    for (int i = 0; i < size; i++) {
        sineWave[i] = (uint8_t)(((sin(i * step) + 1.0) / 2.0) * 255); // Scale sine values to range 0-255
    }

    return sineWave;
}


