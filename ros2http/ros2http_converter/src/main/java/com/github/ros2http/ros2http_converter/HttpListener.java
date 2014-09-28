package com.github.ros2http.ros2http_converter;

import java.io.BufferedWriter;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStreamWriter;
import java.net.HttpURLConnection;
import java.net.URL;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class HttpListener extends AbstractNodeMain {

	protected String url_string = "http://192.168.97.155:80" ;
	protected boolean connected ;
	protected URL url ;
	private HttpURLConnection connection ;
	private BufferedWriter writer ;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("ros2http/http_listener");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		//this.openConnection(this.url_string) ;
		
		final Publisher<std_msgs.String> publisher =
		        connectedNode.newPublisher("ros2http/http_listener/reponse", std_msgs.String._TYPE);
		
		Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber(
				"ros2http/http_listener/json_string", std_msgs.String._TYPE);
		
		subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				long start = System.currentTimeMillis() ;
				if ( HttpListener.this.connected || HttpListener.this.openConnection(HttpListener.this.url_string, message.getData())) {
					String res = HttpListener.this.postConnection(message.getData()) ;
					std_msgs.String ros_res = publisher.newMessage() ;
					ros_res.setData(res) ;
					publisher.publish(ros_res) ;
				} else {
					std_msgs.String ros_res = publisher.newMessage() ;
					ros_res.setData("connection refused: " + HttpListener.this.url_string) ;
					publisher.publish(ros_res) ;
				}
				// System.out.println(" --- time: " + (System.currentTimeMillis()-start) + "[ms]") ;
			}
		}, 1);
	}
	
	@Override
	public void onShutdown(Node node){
		this.closeConnection() ;
	}
	
	protected boolean openConnection(String url, String data) {
		System.out.println("[openConnection] " + url) ;
		try {
			this.url = new URL(url);
			this.connection = (HttpURLConnection) this.url.openConnection();
			this.connection.setDoOutput(true);
			this.connection.setDoInput(true);
//			this.connection.setDoInput(true) ;
			this.connection.setRequestMethod("POST");
			this.connection.setRequestProperty("Content-type", "application/json; charset=utf-8");
			this.connection.setUseCaches(false) ;
			this.connection.setDefaultUseCaches(false);
			this.connection.setConnectTimeout(100) ;
//			this.connection.connect() ;
//			for (String key : this.connection.getHeaderFields().keySet()){
//				System.out.println("-- " + key) ;
//				for ( String val : this.connection.getHeaderFields().get(key)){
//					System.out.println("---- " + val) ;
//				}
//			}
			return (this.connected = true);
		} catch (Exception e) {
			e.printStackTrace() ;
			return (this.connected = false);
		}
	}
	
	protected void closeConnection(){
		if ( this.writer != null ){
			try {
				this.writer.close() ;
				this.writer = null ;
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		if (this.connection != null) {
			this.connection.disconnect();
			this.connection = null ;
		}
		this.connected = false ;
	}
	
	protected String readConnection() throws IOException{
		String ret = "" ;
		if (this.connection.getResponseCode() == HttpURLConnection.HTTP_OK) {
			try {
//				InputStreamReader isr = new InputStreamReader(
//						this.connection.getInputStream(), "UTF-8");
//				BufferedReader reader = new BufferedReader(isr);
//				String line;
//				while ((line = reader.readLine()) != null) {
//					line = line.trim() ;
//					if ( line.startsWith("{") ){
//						ret += line ;
//						ret += "\n" ;
//					}
//				}
//				if ( ret.length() == 0 ){
//					ret = "post without error: " ;
//				}
//				reader.close() ;
//				isr.close() ;
				
				ret = findJsonString(this.connection.getInputStream()) ;
			} catch (IOException e1) {
				System.out.println("[postConnection] input stream error") ;
				e1.printStackTrace() ;
				try {
					ret = "ng: " ;
//					InputStreamReader isr = new InputStreamReader(
//							this.connection.getErrorStream(), "UTF-8");
//					BufferedReader reader = new BufferedReader(isr);
//					String line;
//					while ((line = reader.readLine()) != null) {
//						ret += line ;
//						ret += "\n" ;
//					}
//					reader.close() ;
//					isr.close() ;
					ret = findJsonString(this.connection.getErrorStream()) ;
				} catch (IOException e2) {
					System.out.println("[postConnection] error stream error") ;
					e2.printStackTrace() ;
					ret = e2.getMessage() ;
				}
			}
		} else {
			ret = "error: http response code=" + this.connection.getResponseCode() ;
		}

		return ret ;
	}
	
	protected String postConnection(String data) {
	    // System.out.println("[postConnection] send " + data) ;
		String ret = "" ;
		try {
			this.writer = new BufferedWriter(new OutputStreamWriter(
					this.connection.getOutputStream(), "UTF-8"));
			// this.writer.write("\r\n\r\n\r\n");
			this.writer.write(data);
			// this.writer.write("\r\n");
			this.writer.flush();
			if ( this.writer != null ){
				try {
					this.writer.close() ;
					this.writer = null ;
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
			ret = this.readConnection() ;
			this.closeConnection() ; // ? 
		} catch (IOException e) {
			ret = e.getMessage() ;
		} finally{
		}
		return ret ;
	}

	// stream util
	public byte[] readAll(InputStream inputStream) throws IOException {
	    ByteArrayOutputStream bout = new ByteArrayOutputStream();
	    byte [] buffer = new byte[4096];
	    int cnt = 1 ;
	    // System.out.println("[readAll]") ;
	    inputStream.read(buffer);
	    while(true) {
	        int len = inputStream.read(buffer);
		// System.out.println("  " + cnt + ">0") ;
	        if(len < 0 || --cnt <= 0) {
	            break;
	        }
	        bout.write(buffer, 0, len);
	    }
	    return bout.toByteArray();
	}
	
	public int headerTail(byte[] data){
		for ( int i=0 ; i<data.length-4 ; i++ ){
			if ( data[i]=='\r' && data[i+1]=='\n' && data[i+2]=='\r' && data[i+3]=='\n'){
				return i+4 ;
			} else if (data[i]=='\n' && data[i+1]=='\n'){
			    return i+2 ;
			}
		}
		System.out.println("[headerTail] header missing") ;
		return 0 ;
	}
	
	// public byte[] findJson(byte[] data, int zero){
	//     ByteArrayOutputStream bout = new ByteArrayOutputStream();
	//     int start=-1, end=-1, depth=0 ;
	//     for ( int i=data.length-1 ; i>=0 ; i-- ){
	//     	if ( data[i] == '}' ){
	//     		if ( depth == 0 ){
	//     			end = i ;
	//     		}
	//     		depth++ ;
	//     	} else if ( data[i] == '{' ){
	//     		depth-- ;
	//     		start = i ;
	//     		if ( depth == 0 ){
	//     			bout.write(data, start, end-start) ;
	//     			break ;
	//     		}
	//     	}
	//     }
	//     return bout.toByteArray();
	// }

	public byte[] findJson(byte[] data, int zero){
	    ByteArrayOutputStream bout = new ByteArrayOutputStream();
	    int start=0, end=data.length-1, depth=0 ;
	    for ( int i=zero ; i<data.length ; i++ ){
	    	if ( data[i] == '{' ){
	    		if ( depth == 0 ){
	    			start = i ;
	    		}
	    		depth++ ;
	    	} else if ( data[i] == '}' ){
	    		depth-- ;
	    		end = i ;
	    		if ( depth == 0 ){
	    			break ;
	    		}
	    	}
	    }
	    bout.write(data, start, end-start+1) ;
	    return bout.toByteArray();
	}
	
	public String findJsonString(InputStream st) throws IOException{
		byte[] data = readAll(st) ;
		int zero = 0; //headerTail(data) ;
		String dataString = new String(data) ;
		// System.out.println("[findJsonString] raw=" + dataString + "\n, header=" + zero) ;
		// System.out.println("  debug " + data.length + " vs " + dataString.length()) ;
		byte[] ret = findJson(data,zero) ;
		return new String(ret) ;
	}
}
