#include <stdio.h>
#include <math.h>
#include <vector>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <cstring>

void nrerror(char error_text[]);
float *vector(long nl, long nh);
void free_vector(float *v, long nl, long nh);
void correl(const float data1[], const float data2[], unsigned long n, float ans[]);
