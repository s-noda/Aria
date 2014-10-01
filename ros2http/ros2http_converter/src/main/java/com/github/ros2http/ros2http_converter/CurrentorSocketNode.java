package com.github.ros2http.ros2http_converter;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.parameter.ParameterTree;

public class CurrentorSocketNode extends SocketListener {

	private final static String nodename = "currentor_socket";
	private Publisher<std_msgs.Float32MultiArray>[] sensor_pub ;
	private Publisher<std_msgs.String> sensor_name_pub ;	
	private Publisher<std_msgs.String> currentor_socket_status ;	
	
	final private static int NOP=0, TRQ=1, POS=2, MOD=3, TMAX=4, TMIN=5;
	private int mode ;
	private float[] requested_data ;
    private long last_request_time;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(CurrentorSocketNode.nodename);
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		//super.onStart(connectedNode);

		ParameterTree params = connectedNode.getParameterTree();
		if ( params.getInteger(this.getDefaultNodeName()+"/ARIA_SOCKET_PORT", -1) > 0 ){
			this.portno = params.getInteger(this.getDefaultNodeName()+"/ARIA_SOCKET_PORT", -1);
			System.out.println("ARIA_SOCKET_PORT="+this.portno);			
		} else {
		    this.portno=1024;
		}
		
		this.mode = CurrentorSocketNode.NOP ;
		this.last_request_time = System.currentTimeMillis();
		
		this.sensor_pub = new Publisher[CurrentorUtil.sensor_names.length];
		int id = 0 ;
		for (String name: CurrentorUtil.sensor_names){
			this.sensor_pub[id++] = 
		        connectedNode.newPublisher(CurrentorSocketNode.nodename + "/sensor_array/" + name, std_msgs.Float32MultiArray._TYPE);
		}
		
		this.sensor_name_pub = connectedNode.newPublisher(CurrentorSocketNode.nodename + "/sensor_array/names", std_msgs.String._TYPE);
		this.currentor_socket_status = connectedNode.newPublisher(CurrentorSocketNode.nodename + "/status", std_msgs.String._TYPE);
		
		Subscriber<std_msgs.Float32MultiArray> trq_sub = connectedNode.newSubscriber(
				CurrentorSocketNode.nodename + "/request/torque_vector", std_msgs.Float32MultiArray._TYPE);
		trq_sub.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>(){
			@Override
			public void onNewMessage(std_msgs.Float32MultiArray arg) {
			    synchronized(CurrentorSocketNode.this){
				System.out.println(" -- torque command received " + (System.currentTimeMillis() - CurrentorSocketNode.this.last_request_time) + "ms");
				CurrentorSocketNode.this.mode = CurrentorSocketNode.TRQ;
				CurrentorSocketNode.this.requested_data = arg.getData() ;
				CurrentorSocketNode.this.last_request_time = System.currentTimeMillis();
			    }
			}
		}, 1) ;
		
		Subscriber<std_msgs.Float32MultiArray> pos_sub = connectedNode.newSubscriber(
				CurrentorSocketNode.nodename + "/request/position_vector", std_msgs.Float32MultiArray._TYPE);
		pos_sub.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>(){
			@Override
			public void onNewMessage(std_msgs.Float32MultiArray arg) {
			    synchronized(CurrentorSocketNode.this){
				System.out.println(" -- position command received " + (System.currentTimeMillis() - CurrentorSocketNode.this.last_request_time) + "ms");
				CurrentorSocketNode.this.mode = CurrentorSocketNode.POS;
				CurrentorSocketNode.this.requested_data = arg.getData() ;
				CurrentorSocketNode.this.last_request_time = System.currentTimeMillis();
			    }
			}
		}, 1) ;
		
		Subscriber<std_msgs.Float32MultiArray> mod_sub = connectedNode.newSubscriber(
				CurrentorSocketNode.nodename + "/request/mode_vector", std_msgs.Float32MultiArray._TYPE);
		mod_sub.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>(){
			@Override
			public void onNewMessage(std_msgs.Float32MultiArray arg) {
			    synchronized(CurrentorSocketNode.this){
				System.out.println(" -- mode command received " + (System.currentTimeMillis() - CurrentorSocketNode.this.last_request_time) + "ms");
				CurrentorSocketNode.this.mode = CurrentorSocketNode.MOD;
				CurrentorSocketNode.this.requested_data = arg.getData() ;
				CurrentorSocketNode.this.last_request_time = System.currentTimeMillis();
			    }
			}
		}, 1) ;

		Subscriber<std_msgs.Float32MultiArray> tmax_sub = connectedNode.newSubscriber(
				CurrentorSocketNode.nodename + "/request/torque_max_vector", std_msgs.Float32MultiArray._TYPE);
		tmax_sub.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>(){
			@Override
			public void onNewMessage(std_msgs.Float32MultiArray arg) {
			    synchronized(CurrentorSocketNode.this){
				CurrentorSocketNode.this.mode = CurrentorSocketNode.TMAX;
				CurrentorSocketNode.this.requested_data = arg.getData() ;
			    }
			}
		}, 1) ;

		Subscriber<std_msgs.Float32MultiArray> tmin_sub = connectedNode.newSubscriber(
				CurrentorSocketNode.nodename + "/request/torque_min_vector", std_msgs.Float32MultiArray._TYPE);
		tmin_sub.addMessageListener(new MessageListener<std_msgs.Float32MultiArray>(){
			@Override
			public void onNewMessage(std_msgs.Float32MultiArray arg) {
			    synchronized(CurrentorSocketNode.this){
				CurrentorSocketNode.this.mode = CurrentorSocketNode.TMIN;
				CurrentorSocketNode.this.requested_data = arg.getData() ;
			    }
			}
		}, 1) ;
		
		connectedNode.executeCancellableLoop( new CancellableLoop(){
			private long last_time = System.currentTimeMillis();
			private long step = 10 ;
			private String default_command = CurrentorUtil.encodeJsonCommand("getValues");
			//private std_msgs.String ros_res = CurrentorSocketNode.this.response_pub.newMessage();
			@Override
			protected void loop() throws InterruptedException {
				synchronized(CurrentorSocketNode.this){
					if (CurrentorSocketNode.this.connected
							|| CurrentorSocketNode.this.openConnection(
									CurrentorSocketNode.this.hostname,
									CurrentorSocketNode.this.portno)) {
						String res;
						String command;
						switch ( CurrentorSocketNode.this.mode ){
						case CurrentorSocketNode.TRQ:
						    command = CurrentorUtil.encodeJsonCommand("setTorques",
												     CurrentorSocketNode.this.requested_data);
						    if ( command == null ){
							command = this.default_command ;
							System.out.println(" -- Torque command rejected/");
						    }
						    res = CurrentorSocketNode.this.postConnection(command);
						    break;
						case CurrentorSocketNode.POS:
						    command = CurrentorUtil.encodeJsonCommand("setPositions",
												     CurrentorSocketNode.this.requested_data);
						    if ( command == null ){
							command = this.default_command ;
							System.out.println(" -- Position command rejected/");
						    }
						    res = CurrentorSocketNode.this.postConnection(command);
						    break;
						case CurrentorSocketNode.MOD:
						    command = CurrentorUtil.encodeJsonCommand("setControlModes",
												     CurrentorSocketNode.this.requested_data);
						    if ( command == null ){
							command = this.default_command ;
							System.out.println(" -- Mode command rejected/");
						    }
						    res = CurrentorSocketNode.this.postConnection(command);
						    break;
						case CurrentorSocketNode.TMAX:
							command = CurrentorUtil.encodeJsonCommand("setMaxLimit",
								     CurrentorSocketNode.this.requested_data);
							if (command == null) {
								command = this.default_command;
								System.out
										.println(" -- TMAX command rejected/");
							}
							res = CurrentorSocketNode.this.postConnection(command);
							break;
						case CurrentorSocketNode.TMIN:		
							command = CurrentorUtil.encodeJsonCommand("setMinLimit",
								     CurrentorSocketNode.this.requested_data);
							if (command == null) {
								command = this.default_command;
								System.out
										.println(" -- TMIN command rejected/");
							}
							res = CurrentorSocketNode.this.postConnection(command);
							break;
						case CurrentorSocketNode.NOP:
						default:
						    res = CurrentorSocketNode.this.postConnection(this.default_command);
						    break;
						}
						CurrentorSocketNode.this.mode = CurrentorSocketNode.NOP;
						//
						if ( CurrentorUtil.decodeJsonCommand(res) ){
							std_msgs.String ros_res = CurrentorSocketNode.this.currentor_socket_status.newMessage();
							ros_res.setData("connection refused: "
									+ CurrentorSocketNode.this.hostname + ":"
									+ CurrentorSocketNode.this.portno);
							CurrentorSocketNode.this.currentor_socket_status.publish(ros_res);
						}
						CurrentorSocketNode.this.publishSensors();
					} else {
						System.out.println("connection refused: "
								+ CurrentorSocketNode.this.hostname + ":"
								+ CurrentorSocketNode.this.portno);
						//std_msgs.String ros_res = CurrentorSocketNode.this.response_pub.newMessage();
						//ros_res.setData("connection refused: "
						//		+ CurrentorSocketNode.this.hostname + ":"
						//		+ CurrentorSocketNode.this.portno);
						//CurrentorSocketNode.this.response_pub.publish(ros_res);
					}
				}
				long now = System.currentTimeMillis() ;
				// System.out.println(" --- time: " + (now - this.last_time) + "[ms]") ;
				if ( step > now - this.last_time ){
					Thread.sleep( step - ( now - this.last_time ) ) ;
					// System.out.println(" --- sleep: " + (step- ( now - this.last_time )) + "[ms]") ;
				} else {
					System.out.println(" --- overslept...") ;
				}
				this.last_time = System.currentTimeMillis() ;
			}
			
			
		}) ;
	}
	
	private void publishSensors(){
		for ( int i=0 ; i<CurrentorUtil.sensor_names.length ; i++ ){
			std_msgs.Float32MultiArray mes = this.sensor_pub[i].newMessage() ;
			mes.setData(CurrentorUtil.sensor_values[i]);
			this.sensor_pub[i].publish(mes);
		}
	}
	
	static class CurrentorUtil {
		static String[] sensor_names = new String[] { "position", "velocity",
							      "temperature", "torque", "voltage", "imu" };
		static int joint_cnt = 30;

	    static byte[] byte_buf = new byte[4*(CurrentorUtil.joint_cnt*(CurrentorUtil.sensor_names.length-1)+6)];
		static float[][] sensor_values = new float[CurrentorUtil.sensor_names.length][];
	    static{
		for ( int i=0; i<sensor_names.length-1 ; i++ ){
		    sensor_values[i] = new float[CurrentorUtil.joint_cnt];
		}
		sensor_values[sensor_names.length-1] = new float[6];
	    }
		
		public static boolean decodeSensorValueString(String in){
			if ( in.length() != CurrentorUtil.byte_buf.length*2 ){
				System.out
						.println("[decodeSensorValueString] invalid input string " + in + " length "
								+ in.length()
								+ " vs "
								+ CurrentorUtil.byte_buf.length*2);
				return false;
				//in = in.substring(0,CurrentorUtil.byte_buf.length*2);
			}
			FloatString.xstring2barray(in, CurrentorUtil.byte_buf);
			FloatString.barray2farray(CurrentorUtil.byte_buf, CurrentorUtil.sensor_values, sensor_names.length-1);
			return true;
		}
		
		public static String encodeFloatString(float[] in){
			if ( in.length != CurrentorUtil.joint_cnt ){
				System.out
						.println("[encodeFloatString] invalid input vector length "
								+ in.length
								+ " vs "
								+ CurrentorUtil.joint_cnt);
				return null;
			}
			byte[] buf = FloatString.farray2barray(in);
			return FloatString.barray2xstring(buf);
		}
		
		public static String encodeJsonCommand(String func ){
			return String.format("{\"method\":\"%s\",\"params\":[0],\"id\":\"1\"}", func) ;
		}
		
		public static String encodeJsonCommand(String func, float[] array ){
		    String param = CurrentorUtil.encodeFloatString(array);
		    if ( param != null ){
			return String.format("{\"method\":\"%s\",\"params\":[%s],\"id\":\"1\"}", func, param);
		    } else {
			return null;
		    }
		}
		
		public static boolean decodeJsonCommand(String json){
			String[] split = json.split(":|,");
			int i = 0 ;
			for ( String str : split ){
				i++ ;
				if ( str.contains("result") ){
					break ;
				}
			}
			if ( i >= split.length || (json = split[i].trim()).length() < 2 ){
				System.out.println("[decodeJsonCommand] result missing in " + json) ;
				return false;
			}
			json = json.substring(1,json.length()-1);
			return CurrentorUtil.decodeSensorValueString(json) ;
		}
		
	}
}
