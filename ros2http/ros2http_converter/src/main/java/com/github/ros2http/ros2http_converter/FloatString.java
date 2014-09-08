package com.github.ros2http.ros2http_converter;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.github.ros2http.ros2http_converter.CurrentorSocketNode.CurrentorUtil;

public class FloatString {

	public static boolean little_endian = true; // micon configuration
	private static ByteBuffer buffer = ByteBuffer.allocate(4);
	static{
		FloatString.setEndian(FloatString.buffer);
	}
	
	public static void setEndian(ByteBuffer buf) {
		if (FloatString.little_endian) {
			buf.order(ByteOrder.LITTLE_ENDIAN);
		} else {
			buf.order(ByteOrder.BIG_ENDIAN);
		}
	}

	public static byte[] farray2barray(float[] in) {
		if (FloatString.buffer.capacity() < in.length * 4) {
			System.out.println("resize buffer size "
					+ FloatString.buffer.capacity() + " -> " + (in.length * 4));
			FloatString.buffer = ByteBuffer.allocate(in.length * 4);
			FloatString.setEndian(FloatString.buffer);
		}
		for (int i = 0; i < in.length; i++) {
//			if (FloatString.little_endian) {
//				FloatString.buffer.putFloat(i * 4, in[in.length-i-1]);
//			} else {
				FloatString.buffer.putFloat(i * 4, in[i]);
//			}
		}
		return FloatString.buffer.array();
	}

    public static void barray2farray(byte[] in, float[][] ret, int joint_sensor_cnt) {
	ByteBuffer buf = ByteBuffer.wrap(in);
	FloatString.setEndian(buf);
	for ( int i=0 ; i<ret[0].length ; i++ ){
	    for ( int j=0 ; j<joint_sensor_cnt ; j++ ){
		ret[j][i] = buf.getFloat(4*(i*joint_sensor_cnt+j));
	    }
	}
	for ( int j=joint_sensor_cnt ; j<ret.length ; j++ ){
	    for ( int i=0 ; i<ret[j].length ; i++ ){
		ret[j][i] = buf.getFloat(4*(j*ret[0].length+i));
	    }
	}
    }

    public static void barray2farray(byte[] in, float[][] ret) {
	ByteBuffer buf = ByteBuffer.wrap(in);
	FloatString.setEndian(buf);
	for ( int i=0 ; i<ret[0].length ; i++ ){
	    for ( int j=0 ; j<ret.length ; j++ ){
		ret[j][i] = buf.getFloat(4*(i*ret.length+j));
	    }
	}
    }

	// public static void barray2farray(byte[] in, float[][] ret) {
	// 	ByteBuffer buf = ByteBuffer.wrap(in);
	// 	FloatString.setEndian(buf);
	// 	int id=0 ;
	// 	for ( int j=0 ; j<ret.length ; j++ ){
	// 	    for ( int i=0 ; i<ret[j].length ; i++ ){
	// 			ret[j][i] = buf.getFloat(4*id);
	// 			id++ ;
	// 		}
	// 	}
	// }
	
	public static void barray2farray(byte[] in, float[] ret) {
		ByteBuffer buf = ByteBuffer.wrap(in);
		FloatString.setEndian(buf);
		for ( int i=0 ; i<ret.length ; i++ ){
			ret[i] = buf.getFloat(i*4);
		}
	}
	
	public static float[] barray2farray(byte[] in) {
		float[] ret = new float[in.length/4];
		barray2farray(in,ret);
		return ret ;
	}

	public static String byte2string(byte in) {
		return String.format("%02x", in);
	}
	
	public static byte string2byte(char big, char lit){
		if ( big >= 'A' ) big = (char)(big - 'A' + 10);
		else if ( big >= 'a' ) big = (char)(big - 'a' + 10);
		else big = (char)(big - '0');
		if ( lit >= 'A' ) lit = (char)(lit - 'A' + 10);
		else if ( lit >= 'a' ) lit = (char)(lit - 'a' + 10);
		else lit = (char)(lit - '0');
		//return (byte)(big*16+lit);
		return (byte) ( big << 4 & 0xF0 | lit & 0x0F);
	}
	
	public static String barray2xstring(byte[] in){
		return FloatString.barray2xstring(in,0,in.length);
	}
	
	public static String barray2xstring(byte[] in, int start, int end) {
		StringBuilder buf = new StringBuilder();
		for (int i=start ; i<end ; i++ ){
			byte b = in[i];
			buf.append(Character.forDigit(b >> 4 & 0xF, 16));
			buf.append(Character.forDigit(b & 0xF, 16));
		//	buf.append(FloatString.x02(b));
		}
		return buf.toString();
	}
	
	public static void xstring2barray(String xstring, byte[] ret){
//		byte[] ret = new byte[xstring.length()/2];
		for ( int i=0 ; i<ret.length ; i++ ){
			ret[i] = string2byte(xstring.charAt(i*2), xstring.charAt(i*2+1));
			// ret[i] = (byte)Integer.parseInt(xstring.substring(i*2, i*2+2));
		}
	}
	
	public static byte[] xstring2barray(String xstring){
		byte[] ret = new byte[xstring.length()/2];
		for ( int i=0 ; i<ret.length ; i++ ){
			ret[i] = string2byte(xstring.charAt(i*2), xstring.charAt(i*2+1));
			// ret[i] = (byte)Integer.parseInt(xstring.substring(i*2, i*2+2));
		}
		return ret ;
	}

	public static boolean randomTest(){
		float[] rand = new float[3];
		for ( int i=0 ; i<rand.length ; i++ ){
			rand[i] = (float)Math.random() ;
		}
		byte[] barray = FloatString.farray2barray(rand);
		String xstring = FloatString.barray2xstring(barray);
		byte[] barray2 = FloatString.xstring2barray(xstring);
		float[] farray2 = FloatString.barray2farray(barray2);
		//
		for ( int i=0 ; i<rand.length; i++ ){
			System.out.print(" " + rand[i] );
		}
		System.out.print("(" + xstring + ") --> ");
		//
		for ( int i=0 ; i<farray2.length; i++ ){
			System.out.print(" " + farray2[i] );
		}
		float dif = 0 ;
		for ( int i=0 ; i<rand.length ; i++ ){
			dif = Math.abs(rand[i] - farray2[i]) ;
		}
		System.out.println("");
		return ( dif < 1e-10 ) ;
	}
	
	public static void main(String[] args) {
		System.out.println("start random test") ;
		boolean res = true ;
		for ( int i=0 ; i<100 ; i++ ){
			res = res && randomTest() ;
		}
		if (res) {
			System.out.println("passed");
		} else {
			System.out.println("error");
		}
		
//		String data = "0000000000000000000000000000000000000000ec609bbf000000000000d84100000000000000000082a63b000000000000d841000000000000000080bc65bd000000000000dc4100000000000000003d0624c0000000000000d8410000000000000000406163bd000000000000d841000000000000000080f738bd000000000000d841000000000000000010b7f93e000000000000d441000000000000000064e208bf000000000000dc410000000000000000b013793e000000000000dc41000000000000000070ed2a3e000000000000dc410000000000000000c04c62be000000000000d8410000000000000000004a873d000000000000dc4100000000000000008486193f000000000000d841000000000000000040b3063d000000000000d4410000000000000000cac6b73f000000000000dc410000000000000000c0893abd000000000000e0410000000000000000009697bc000000000000e04100000000000000004c69683f000000000000d4410000000000000000d0fbafbe000000000000d8410000000000000000a0b0f5be000000000000d84100000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000009";
//		String json = "{\"result\": [" + data + "], \"id\": \"1\"}";
//		// System.out.println(" decode json command " + json);
//		CurrentorUtil.decodeJsonCommand(json);
//		CurrentorUtil.encodeJsonCommand("setTorques2",CurrentorUtil.sensor_values[2]);
//		for ( int i=0 ; i<CurrentorUtil.joint_cnt ; i++ ){
//			System.out.print(" " + CurrentorUtil.sensor_values[2][i]) ;
//		}
//		System.out.println("");
//		System.out.println(CurrentorUtil.encodeJsonCommand("setTorques2",CurrentorUtil.sensor_values[2]));
	}
}
