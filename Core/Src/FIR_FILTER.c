/*
 * FIR_FILTER.c
 *
 *  Created on: May 1, 2024
 *      Author: oguzk
 */


#include "FIR_FILTER.h"


static float FIR_IMPULSE_RESPONSE [FIR_FILTER_LENGHT] = {
		-0.009904741547314157,
		-0.0427633534696972,
		-0.022060373130520804,
		0.05396514614572738,
		0.0025069840922072564,
		-0.11518277931718884,
		0.08652811485817445,
		0.49826460133664197,
		0.49826460133664197,
		0.08652811485817445,
		-0.11518277931718884,
		0.0025069840922072564,
		0.05396514614572738,
		-0.022060373130520804,
		-0.0427633534696972,
		-0.009904741547314157

};


static float MAF_IMPULSE_RESPONSE [4] = {0.25f , 0.25f , 0.25f , 0.25f};



void FIRFilter_Init(FIRFilter *fir)
{
	for (uint8_t n = 0; n< FIR_FILTER_LENGHT; n++)
	{
		fir->buf[n] =0.0f; // filtre buffer temizleme

	}

	fir->bufIndex =0; // index reset
	fir->out = 0; // clear output
}




float FIRFilter_Update(FIRFilter *fir , float inp)
{
	// son gelen veriyi buffer'a kaydet
	fir->buf[fir->bufIndex] =inp;

	// buffer indexini arttır
	fir->bufIndex++;

	// index uzunluğu taşması kontrolü

	if(fir->bufIndex == FIR_FILTER_LENGHT) fir->bufIndex=0;

	fir->out =0;

	uint8_t sumIndex = fir->bufIndex;

	for (uint8_t n = 0; n< FIR_FILTER_LENGHT; n++)
	{
		// index kontrol
		if(sumIndex >0) sumIndex --;
		else sumIndex = FIR_FILTER_LENGHT-1;

		// convulution toplam kısmı
		fir->out += FIR_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];

	}

	return fir->out;

}



void MAFilter_Init(FIRFilter *fir)
{
	for (uint8_t n = 0; n< MAV_FILTER_LENGHT; n++)
	{
		fir->buf[n] =0.0f; // filtre buffer temizleme

	}

	fir->bufIndex =0; // index reset
	fir->out = 0; // clear output
}



float MAVFilter_Update(FIRFilter *fir , float inp)
{
	// son gelen veriyi buffer'a kaydet
	fir->buf[fir->bufIndex] =inp;

	// buffer indexini arttır
	fir->bufIndex++;

	// index uzunluğu taşması kontrolü

	if(fir->bufIndex == MAV_FILTER_LENGHT) fir->bufIndex=0;

	fir->out =0;

	uint8_t sumIndex = fir->bufIndex;

	for (uint8_t n = 0; n< MAV_FILTER_LENGHT; n++)
	{
		// index kontrol
		if(sumIndex >0) sumIndex --;
		else sumIndex = MAV_FILTER_LENGHT-1;

		// convulution toplam kısmı
		fir->out += MAF_IMPULSE_RESPONSE[n] * fir->buf[sumIndex];

	}

	return fir->out;

}
