float fir_basic(float input, int ntaps, float z[])  // Arrays are passed by pointers by default. 
{
    
    int ii;
    float accum;
    
    /* store input at the beginning of the delay line */
    z[0] = input;

    /* calc FIR */
    accum = 0;
    for (ii = 0; ii < ntaps; ii++) {
        accum += z[ii];
//      accum += h[ii] * z[ii];
    }

    /* shift delay line */
    for (ii = ntaps - 2; ii >= 0; ii--) {
        z[ii + 1] = z[ii];
    }

    return (accum/float(ntaps));
}


// Clears the given filter line
void clearFIR(int ntaps, float z[])      
{
    int ii;
    for (ii = 0; ii < ntaps; ii++) {
        z[ii] = 0;
    }
}


// Sets the filter line to a given value
void setFIR(int ntaps, float z[], float SetVal)
{
    int ii;
    for (ii = 0; ii < ntaps; ii++) {
        z[ii] = SetVal;
    }
}


