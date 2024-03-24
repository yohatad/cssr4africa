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

void correl(float data1[], float data2[], unsigned long n, float ans[])
{
    void realft(float data[], unsigned long n, int isign);
    void twofft(float data1[], float data2[], float fft1[], float fft2[], unsigned long n);
    
    unsigned long no2, i;
    float dum, *fft;

    fft = vector(1, n << 1); // Allocate memory for FFT arrays
    
    twofft(data1, data2, fft, ans, n); // Transform both data vectors at once
    
    no2 = n >> 1; // Normalization factor for inverse FFT
    
    for (i = 2; i <= n + 2; i += 2) {
        ans[i - 1] = (fft[i - 1] * (dum = ans[i - 1]) + fft[i] * ans[i]) / no2; // Multiply to find FFT of their correlation
        ans[i] = (fft[i] * dum - fft[i - 1] * ans[i]) / no2;
    }
    
    ans[2] = ans[n + 1]; // Pack first and last elements into one
    
    realft(ans, n, -1); // Inverse transform gives correlation
    
    free_vector(fft, 1, n << 1); // Free memory used for FFT arrays
}


void twofft(float data1[], float data2[], float fft1[], float fft2[], unsigned long n)
{
    void four1(float data[], unsigned long nn, int isign);
    unsigned long nn3, nn2, jj, j;
    float rep, rem, aip, aim;

    nn3 = 1 + (nn2 = 2 + n + n); // Calculate sizes for intermediate arrays

    // Pack the two real arrays into one complex array
    for (j = 1, jj = 2; j <= n; j++, jj += 2) {
        fft1[jj - 1] = data1[j];
        fft1[jj] = data2[j];
    }

    four1(fft1, n, 1); // Transform the complex array using FFT

    fft2[1] = fft1[2];
    fft1[2] = fft2[2] = 0.0;

    // Use symmetries to separate the two transformed forms
    for (j = 3; j <= n + 1; j += 2) {
        rep = 0.5 * (fft1[j] + fft1[nn2 - j]); // Real part
        rem = 0.5 * (fft1[j] - fft1[nn2 - j]); // Real part
        aip = 0.5 * (fft1[j + 1] + fft1[nn3 - j]); // Imaginary part
        aim = 0.5 * (fft1[j + 1] - fft1[nn3 - j]); // Imaginary part
        
        // Ship them out in two complex arrays
        fft1[j] = rep;
        fft1[j + 1] = aim;
        fft1[nn2 - j] = rep;
        fft1[nn3 - j] = -aim;
        fft2[j] = aip;
        fft2[j + 1] = -rem;
        fft2[nn2 - j] = aip;
        fft2[nn3 - j] = rem;
    }
}

void realft(float data[], unsigned long n, int isign)
{
    void four1(float data[], unsigned long nn, int isign);
    unsigned long i, i1, i2, i3, i4, np3;
    float c1 = 0.5, c2, h1r, h1i, h2r, h2i;
    double wr, wi, wpr, wpi, wtemp, theta;

    theta = 3.141592653589793 / (double)(n >> 1); // Initialize the trigonometric recurrence

    if (isign == 1) {
        c2 = -0.5;
        four1(data, n >> 1, 1); // Perform the forward transform
    } else {
        c2 = 0.5;
        theta = -theta; // Set up for an inverse transform
    }

    wtemp = sin(0.5 * theta);
    wpr = -2.0 * wtemp * wtemp;
    wpi = sin(theta);
    wr = 1.0 + wpr;
    wi = wpi;
    np3 = n + 3;

    // Separate the two transforms and recombine them to get the true transform of the original real data
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
        wr = (wtemp = wr) * wpr - wi * wpi + wr; // Trigonometric recurrence
        wi = wi * wpr + wtemp * wpi + wi;
    }

    if (isign == 1) {
        data[1] = (h1r = data[1]) + data[2]; // Squeeze the first and last data components together
        data[2] = h1r - data[2];
    } else {
        data[1] = c1 * ((h1r = data[1]) + data[2]);
        data[2] = c1 * (h1r - data[2]);
        four1(data, n >> 1, -1); // Perform the inverse transform for the case isign=-1
    }
}



#define SWAP(a, b) tempr = (a); (a) = (b); (b) = tempr
void four1(float data[], unsigned long nn, int isign)
{
    unsigned long n, mmax, m, j, istep, i;
    double wtemp, wr, wpr, wpi, wi, theta;
    float tempr, tempi;

    n = nn << 1; // Calculate the total data length

    j = 1;

    // Reorder the data using bit reversal
    for (i = 1; i < n; i += 2) {
        if (j > i) {
            SWAP(data[j], data[i]); // Exchange the two complex numbers
            SWAP(data[j + 1], data[i + 1]);
        }
        m = nn;
        while (m >= 2 && j > m) {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    mmax = 2;

    // Apply the Danielson-Lanczos section of the routine
    while (n > mmax) { // Outer loop executed log2 nn times
        istep = mmax << 1;
        theta = isign * (6.28318530717959 / mmax); // Initialize the trigonometric recurrence
        wtemp = sin(0.5 * theta);
        wpr = -2.0 * wtemp * wtemp;
        wpi = sin(theta);
        wr = 1.0;
        wi = 0.0;

        // Nested inner loops for the Danielson-Lanczos formula
        for (m = 1; m < mmax; m += 2) {
            for (i = m; i <= n; i += istep) {
                j = i + mmax;
                // Danielson-Lanczos formula
                tempr = wr * data[j] - wi * data[j + 1];
                tempi = wr * data[j + 1] + wi * data[j];
                data[j] = data[i] - tempr;
                data[j + 1] = data[i + 1] - tempi;
                data[i] += tempr;
                data[i + 1] += tempi;
            }
            wr = (wtemp = wr) * wpr - wi * wpi + wr; // Trigonometric recurrence
            wi = wi * wpr + wtemp * wpi + wi;
        }
        mmax = istep;
    }
}



