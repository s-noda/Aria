#include <stdio.h>
#include <stdlib.h>

int big_endian=0 ;

int endian_check(){
  unsigned int x = 1;
  unsigned char* x_buf = (unsigned char*)&x;
  printf("%d = %02x%02x%02x%02x --> ",x, x_buf[0],x_buf[1],x_buf[2],x_buf[3]);
  if ( x_buf[0] ){
    printf("little endian\n");
    return 0 ;
  } else {
    printf("big endian\n");
    return 1 ;
  }
}

void hello_world_test(){
  printf("hello world\n") ;
}

void float_to_string(float in, char* out, int start){
  unsigned char* casted_float = (unsigned char*)&in ;
  int i;
  for (i=0 ; i<sizeof(float) ; i++){
    if ( big_endian ){
      sprintf(out+start+i*2, "%02x",casted_float[sizeof(float)-i-1]) ;
    } else {
      sprintf(out+start+i*2, "%02x",casted_float[i]) ;
    }
  }
}

void float_to_string_for_eus(double in_, char* out, int start){
  float in = (float)in_ ;
  float_to_string(in,out,start) ;
}

float string_to_float(char* in){
  // printf("string_to_float %s\n", in) ;
  unsigned char *tmp = (unsigned char*)malloc(sizeof(float)) ;
  int i=0;
  float *ret;
  for (; i<sizeof(float) ; i++){
    unsigned char big, lit ;
    if ( in[2*i] >= 'a' ){
      big = in[2*i] - 'a' + 10;
    } else {
      big = in[2*i] - '0' ;
    }
    if ( in[2*i+1] >= 'a' ){
      lit = in[2*i+1] - 'a' + 10;
    } else {
      lit = in[2*i+1] - '0' ;
    }
    if (big_endian){
      tmp[sizeof(float)-i-1] = big*16+lit;
    } else {
      tmp[i] = big * 16 + lit ;
    }
    // printf( "%02x%02x = %c\n", (in[2*i]-'0'), (in[2*i+1]-'0'), tmp[i]);
  }
  ret = (float*)tmp ;
  // printf("float string=%s = %f\n",in, *ret) ;
  return *ret ;
}

void string_to_float_for_eus(char* in, double* buf){
  buf[0] = string_to_float(in) ;
}
