#include "cordiccart2pol.h"
#include "math.h"
data_t Kvalues[NO_ITER] = {1,	0.500000000000000,	0.250000000000000,	0.125000000000000,	0.0625000000000000,	0.0312500000000000,	0.0156250000000000,	0.00781250000000000,	0.00390625000000000,	0.00195312500000000,	0.000976562500000000,	0.000488281250000000,	0.000244140625000000,	0.000122070312500000,	6.10351562500000e-05,	3.05175781250000e-05};

data_t angles[NO_ITER] = {0.785398163397448,	0.463647609000806,	0.244978663126864,	0.124354994546761,	0.0624188099959574,	0.0312398334302683,	0.0156237286204768,	0.00781234106010111,	0.00390623013196697,	0.00195312251647882,	0.000976562189559320,	0.000488281211194898,	0.000244140620149362,	0.000122070311893670,	6.10351561742088e-05,	3.05175781155261e-05};


void cordiccart2pol(data_t x, data_t y, data_t * r,  data_t * theta)
{
#pragma HLS INTERFACE s_axilite port=x bundle=CTRL
#pragma HLS INTERFACE s_axilite port=y bundle=CTRL
#pragma HLS INTERFACE s_axilite port=r bundle=CTRL
#pragma HLS INTERFACE s_axilite port=theta bundle=CTRL
#pragma HLS INTERFACE s_axilite port=return

	data_t current_cos= x ;
	data_t current_sin= y ;
	data_t angle = 0;
	data_t pi = 3.1415926;
	coef_t flag;

	if(current_cos < 0){
		current_cos=-current_cos;
		if(current_sin<0){
			flag = 2;
		}else{
			flag = 1;
		}
	}else{
		flag = 0;
	}

	for (int i=0; i<NO_ITER;i++){
#pragma HLS PIPELINE ii=1
		data_t temp_cos = current_cos ;
		coef_t sigma = (current_sin<0)?1:-1;

		//Perform the rotation
		current_cos = current_cos - current_sin*sigma*Kvalues[i];
		current_sin = current_sin + temp_cos*sigma*Kvalues[i];
		//Determine the new theta
		angle = angle - sigma * angles[i];
	}
	// Set the final r and theta values
	*r = current_cos * 0.607252935;

	if(flag==2){
		angle = -pi- angle;
	}else if(flag==1){
		angle = pi- angle;
	}else{
		angle = angle;
	}
	*theta = angle;
}

