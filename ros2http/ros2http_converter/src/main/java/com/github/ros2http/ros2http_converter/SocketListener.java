package com.github.ros2http.ros2http_converter;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.net.SocketTimeoutException;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class SocketListener extends HttpListener {

	protected String hostname = "192.168.97.155" ;
	protected int portno = 1023 ;
	
	protected InetAddress addr ;
	protected Socket socket ;
	
	protected boolean connected ;
	protected OutputStream writer ;
	protected InputStream reader ;
	
	private Publisher<std_msgs.String> response_pub;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("ros2http/socket_listener");
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		
		ParameterTree params = connectedNode.getParameterTree();
		if ( params.getInteger("ARIA_SOCKET_PORT", -1) > 0 ){
			this.portno = params.getInteger("ARIA_SOCKET_PORT", -1);
			System.out.println("ARIA_SOCKET_PORT="+this.portno);			
		}
		
		this.response_pub =
		        connectedNode.newPublisher("ros2http/socket_listener/reponse", std_msgs.String._TYPE);
		
		Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber(
				"ros2http/socket_listener/json_string", std_msgs.String._TYPE);
		
		subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				long start = System.currentTimeMillis() ;
				synchronized(SocketListener.this){
					if (SocketListener.this.connected
							|| SocketListener.this.openConnection(
									SocketListener.this.hostname,
									SocketListener.this.portno)) {
						String res = SocketListener.this.postConnection(message
								.getData());
						std_msgs.String ros_res = SocketListener.this.response_pub.newMessage();
						ros_res.setData(res);
						SocketListener.this.response_pub.publish(ros_res);
					} else {
						std_msgs.String ros_res = SocketListener.this.response_pub.newMessage();
						ros_res.setData("connection refused: "
								+ SocketListener.this.hostname + ":"
								+ SocketListener.this.portno);
						SocketListener.this.response_pub.publish(ros_res);
					}
				}
				System.out.println(" --- time: " + (System.currentTimeMillis()-start) + "[ms]") ;
			}
		}, 1);
	}
	
	@Override
	public void onShutdown(Node node){
		this.closeConnection() ;
	}
	
	protected boolean openConnection(String hostname, int portno) {
		System.out.println("[openConnection] " + hostname + ":" + portno) ;
		try {
			this.addr = InetAddress.getByName(hostname);
			this.socket = new Socket(this.addr, portno);
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
		if ( this.reader != null ){
			try {
				this.reader.close() ;
				this.reader = null ;
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		this.connected = false ;
	}
	
	protected String readConnection() throws IOException{
		String ret = "" ;
		System.out.println("[readConnection] read " ) ;
		try {
			this.socket.setSoTimeout(3);
			ret = findJsonString((this.reader = this.socket.getInputStream()));
			// this.reader.close() ;
		} catch (IOException e1) {
			System.out.println("[readConnection] input stream error");
			e1.printStackTrace();
		}
		return ret ;
	}
	
	protected String postConnection(String data) {
		System.out.println("[postConnection] send " + data) ;
		String ret = "" ;
		try {
//			this.writer = new BufferedWriter(new OutputStreamWriter(
//					this.socket.getOutputStream(), "UTF-8"));
			this.writer = this.socket.getOutputStream();
//			this.writer.write("POST: HTTP/1.0\r\n");
//			this.writer.write("Content-Length: " + data.length() + "\r\n");
//			this.writer.write("Content-Type: application/x-www-form-urlencoded\r\n");
//			this.writer.write("\r\n");
//			this.writer.write(data);
			this.writer.write(data.getBytes());
			// this.writer.write("\n");
			this.writer.flush();
			this.writer.flush();
//			this.writer.close();
			ret = this.readConnection() ;
			//this.closeConnection() ;
		} catch (IOException e) {
			ret = e.getMessage() ;
		} finally{
		}
		return ret ;
	}

	// stream util
	public byte[] readAll(InputStream inputStream) throws IOException {
	    ByteArrayOutputStream bout = new ByteArrayOutputStream();
	    byte [] buffer = new byte[5000];
	    int cnt = 2;
	    int len ;
	    System.out.println("[readAll]") ;
	    try {
		while(true) {
		    len = inputStream.read(buffer);
		    System.out.println("  " + cnt + ">0 (" + len + ")") ;
		    if(len < 0 || --cnt <= 0) {
			break;
		    }
		    bout.write(buffer, 0, len);
		}
	    } catch (SocketTimeoutException ex) {
		System.out.println( "[readAll] Timeout!!" );
		// ex.printStackTrace() ;
	    }
	    return bout.toByteArray();
	}
}
