#include <stdio.h>
#include <stdlib.h>

#include "float_string.c"

int main(int argc, char** argv){
  float f = atof(argv[1]);
  char buf[sizeof(float)*2] ;
  big_endian = endian_check();
  float_to_string_for_eus((double)f,buf,0) ;
  float f2 = string_to_float(buf);
  printf("%f(%s) vs %f = %f\n", f, buf, f2, (f-f2)) ;
  return 0;
}
