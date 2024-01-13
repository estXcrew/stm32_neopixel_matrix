/*
 * pixel_shape_masks.h
 *
 *  Created on: Nov 11, 2023
 *      Author: raul
 */
#include <stdint.h>

#ifndef INC_PIXEL_SHAPE_MASKS_H_
#define INC_PIXEL_SHAPE_MASKS_H_

typedef struct {
    uint8_t line[8];
} matrix_shape;

matrix_shape matrix_one = {{
	0b00011100,
	0b00111100,
	0b01111100,
	0b01101100,
	0b00001100,
	0b00001100,
	0b00001100,
	0b00001100
}};

matrix_shape matrix_alt = {{
	0b00000000,
	0b11111111,
	0b00000000,
	0b11111111,
	0b00000000,
	0b11111111,
	0b00000000,
	0b11111111
}};

matrix_shape matrix_alt_2 = {{
	0b11111111,
	0b00000000,
	0b11111111,
	0b00000000,
	0b11111111,
	0b00000000,
	0b11111111,
	0b00000000
}};

matrix_shape matrix_square = {{
	0b00000000,
	0b00111110,
	0b00100010,
	0b00100010,
	0b00111110,
	0b00000000,
	0b00000000,
	0b00000000
}};

matrix_shape matrix_clear = {{
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000000
}};

matrix_shape matrix_left = {{
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000,
	0b10000000
}};

matrix_shape matrix_right = {{
	0b00000001,
	0b00000001,
	0b00000001,
	0b00000001,
	0b00000001,
	0b00000001,
	0b00000001,
	0b00000001
}};

matrix_shape matrix_pillars = {{
	0b10010100,
	0b10010100,
	0b10010100,
	0b10010100,
	0b10010100,
	0b10010100,
	0b10010100,
	0b10010100
}};

matrix_shape matrix_checkers = {{
	0b11111111,
	0b10101011,
	0b01010101,
	0b10101011,
	0b01010101,
	0b10101011,
	0b01010101,
	0b11111111
}};


#endif /* INC_PIXEL_SHAPE_MASKS_H_ */
