/************************************************************************************************
/** @file soundDetection.cpp
  *
  * Version 1.0
  * 
  * 09/08/2023
  *
  * \author 
  * 
  * <Yohannes Haile>, <CMU-Africa>  
  * 
  * Copyright (C) 2023 CSS4Africa
  * 
  */
/* 
 * Copyright (C) 2016 CSSR4A Consortium
 * FP7 Project 611391 co-funded by the European Commission
 *
 * Author:  Yohannes Haile, Carnegie Mellon Univesity Africa
 * Email:   yohanneh@andrew.cmu.edu
 * Website: www.cssr4africa.com
 * 
 * This file is part of CSS4A.
 * 
 */
  
/*
 * Audit Trail
 * -----------
 * 10/08/16  First version validated (Yohannes Haile) 
*  
* Function: Cross correlation
*
*	code is taken from a book (Numerical Recipes in C The Art of Scientific Computing) 
*	it is used to compute the correlation between two sets of data. it includes other 
*	fuctions such as FFT.
* 
*  Input parameters 
*
*     data1          the samples of the left microphone
*
*     data2          the samples of the right microphone
*          
*	  n				 number of samples in data1 and data2 arrays
*
*  Output parameters:
*
*     ans			 the result of the correlation				  
*                         
* 
*************************************************************************************************/

#include "util.h"
#include <vector>
#include <cmath>

void four1(float data[], unsigned long nn, int isign);
void realft(float data[], unsigned long n, int isign);
void twofft(const float data1[], const float data2[], float fft1[], float fft2[], unsigned long n);

void correl(const float data1[], const float data2[], unsigned long n, float ans[])
{
    unsigned long no2 = n >> 1;
    std::vector<float> fft(n << 1);

    // Perform FFT on both data vectors
    twofft(data1, data2, fft.data(), ans, n);

    // Multiply to find FFT of their correlation
    for (unsigned long i = 2; i <= n + 2; i += 2) {
        float temp = ans[i - 1];
        ans[i - 1] = (fft[i - 1] * temp + fft[i] * ans[i]) / no2;
        ans[i] = (fft[i] * temp - fft[i - 1] * ans[i]) / no2;
    }

    // Pack first and last elements into one
    ans[2] = ans[n + 1];

    // Inverse transform gives correlation
    realft(ans, n, -1);
}

void twofft(const float data1[], const float data2[], float fft1[], float fft2[], unsigned long n)
{
    unsigned long nn3 = 1 + (2 + n + n); // Calculate sizes for intermediate arrays

    // Pack the two real arrays into one complex array
    for (unsigned long j = 1, jj = 2; j <= n; j++, jj += 2) {
        fft1[jj - 1] = data1[j];
        fft1[jj] = data2[j];
    }

    // Transform the complex array using FFT
    four1(fft1, n, 1);

    fft2[1] = fft1[2];
    fft1[2] = fft2[2] = 0.0;

    // Use symmetries to separate the two transformed forms
    for (unsigned long j = 3; j <= n + 1; j += 2) {
        float rep = 0.5f * (fft1[j] + fft1[nn3 - j]);
        float rem = 0.5f * (fft1[j] - fft1[nn3 - j]);
        float aip = 0.5f * (fft1[j + 1] + fft1[nn3 - j]);
        float aim = 0.5f * (fft1[j + 1] - fft1[nn3 - j]);

        fft1[j] = rep;
        fft1[j + 1] = aim;
        fft1[nn3 - j] = rep;
        fft1[nn3 - j] = -aim;
        fft2[j] = aip;
        fft2[j + 1] = -rem;
        fft2[nn3 - j] = aip;
        fft2[nn3 - j] = rem;
    }
}

void realft(float data[], unsigned long n, int isign)
{
    unsigned long i, i1, i2, i3, i4, np3;
    float c1 = 0.5, c2, h1r, h1i, h2r, h2i;
    double wr, wi, wpr, wpi, wtemp, theta;

    theta = M_PI / (double)(n >> 1);

    if (isign == 1) {
        c2 = -0.5;
        four1(data, n >> 1, 1);
    } else {
        c2 = 0.5;
        theta = -theta;
    }

    wtemp = sin(0.5 * theta);
    wpr = -2.0 * wtemp * wtemp;
    wpi = sin(theta);
    wr = 1.0 + wpr;
    wi = wpi;
    np3 = n + 3;

    for (i = 2; i <= (n >> 2); i++) {
        i4 = 1 + (i3 = np3 - (i2 = 1 + (i1 = i + i - 1)));
        h1r = c1 * (data[i1] + data[i3]);
        h1i = c1 * (data[i2] - data[i4]);
        h2r = -c2 * (data[i2] + data[i4]);
        h2i = c2 * (data[i1] - data[i3]);
        data[i1] = h1r + wr * h2r - wi * h2i;
        data[i2] = h1i + wr * h2i + wi * h2r;
        data[i3] = h1r - wr * h2r + wi * h2i;
        data[i4] = -h1i + wr * h2i + wi * h2r;
        wr = (wtemp = wr) * wpr - wi * wpi + wr;
        wi = wi * wpr + wtemp * wpi + wi;
    }

    if (isign == 1) {
        data[1] = (h1r = data[1]) + data[2];
        data[2] = h1r - data[2];
    } else {
        data[1] = c1 * ((h1r = data[1]) + data[2]);
        data[2] = c1 * (h1r - data[2]);
        four1(data, n >> 1, -1);
    }
}

#define SWAP(a, b) { float temp = (a); (a) = (b); (b) = temp; }

void four1(float data[], unsigned long nn, int isign)
{
    unsigned long n = nn << 1;
    unsigned long j = 1;

    for (unsigned long i = 1; i < n; i += 2) {
        if (j > i) {
            SWAP(data[j], data[i]);
            SWAP(data[j + 1], data[i + 1]);
        }
        unsigned long m = nn;
        while (m >= 2 && j > m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    unsigned long mmax = 2;
    while (n > mmax) {
        unsigned long istep = mmax << 1;
        double theta = isign * (2.0 * M_PI / mmax);
        double wtemp = sin(0.5 * theta);
        double wpr = -2.0 * wtemp * wtemp;
        double wpi = sin(theta);
        double wr = 1.0;
        double wi = 0.0;

        for (unsigned long m = 1; m < mmax; m += 2) {
            for (unsigned long i = m; i <= n; i += istep) {
                j = i + mmax;
                float tempr = wr * data[j] - wi * data[j + 1];
                float tempi = wr * data[j + 1] + wi * data[j];
                data[j] = data[i] - tempr;
                data[j + 1] = data[i + 1] - tempi;
                data[i] += tempr;
                data[i + 1] += tempi;
            }
            wr = (wtemp = wr) * wpr - wi * wpi + wr;
            wi = wi * wpr + wtemp * wpi + wi;
        }
        mmax = istep;
    }
}
