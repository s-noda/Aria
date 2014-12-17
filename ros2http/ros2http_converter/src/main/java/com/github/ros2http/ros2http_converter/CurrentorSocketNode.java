package com.github.ros2http.ros2http_converter;

import java.util.ArrayList;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.concurrent.CancellableLoop;
import org.ros.node.parameter.ParameterTree;
import org.ros.exception.ParameterNotFoundException;
import org.ros.exception.ParameterClassCastException;

public class CurrentorSocketNode extends SocketListener {

	private final static String nodename = "currentor_socket";
	private Publisher<std_msgs.Float32MultiArray>[] sensor_pub;
	private Publisher<std_msgs.String> sensor_name_pub;
	private Publisher<std_msgs.String> currentor_socket_status;

	final private static int NOP = 0, TRQ = 1, POS = 2, MOD = 3, TMAX = 4,
	    TMIN = 5, WHL = 6, PID = 7, CTV = 8, DTQ = 9;
	//private int mode;
	//private float[] requested_data;
	private long last_request_time;
	private long com_step_time;
	
	private ArrayList<CurrentorVectorCommand> vectorCommand = new ArrayList<CurrentorVectorCommand>();

	public class CurrentorVectorCommand {
		public int mode;
		public float[] requested_data;

		public CurrentorVectorCommand(int mode, float[] rd) {
			this.mode = mode;
			this.requested_data = rd;
		}

	    public void dump(){
		int size = -1;
		if ( this.requested_data != null ) size = this.requested_data.length;
		System.out.println("  -- " + this.mode + " len:" + size);
	    }
	}

	public void vectorCommandEnqueue(int mode, float[] rd){
		this.vectorCommand.add(new CurrentorVectorCommand(mode,rd));
		while ( this.vectorCommand.size() > 5 ){
			this.vectorCommand.remove(0);
		}
	}

	public CurrentorVectorCommand vectorCommandPop(){
		CurrentorVectorCommand ret = null;
		if ( this.vectorCommand.size() > 0 ) ret = this.vectorCommand.remove(0);
		return ret;
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of(CurrentorSocketNode.nodename);
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
		// super.onStart(connectedNode);

		ParameterTree params = connectedNode.getParameterTree();
		// hostname
		try {
			this.hostname = params.getString(connectedNode.getName()
					+ "/ARIA_SOCKET_HOSTNAME", "192.168.97.155");
		} catch (ParameterNotFoundException e) {
			System.err.println("Parameter Not Found: " + e.getMessage());
		} catch (ParameterClassCastException e) {
			System.err.println("Cast Failed: " + e.getMessage());
		}
		System.out.println("[CurrentorSocket] get aria_hostname="
				+ this.hostname + " from " + connectedNode.getName()
				+ "/ARIA_SOCKET_HOSTNAME");
		// portno
		try {
			this.portno = params.getInteger(connectedNode.getName()
					+ "/ARIA_SOCKET_PORT", 1024);
		} catch (ParameterNotFoundException e) {
			System.err.println("Parameter Not Found: " + e.getMessage());
		} catch (ParameterClassCastException e) {
			System.err.println("Cast Failed: " + e.getMessage());
		}
		System.out.println("[CurrentorSocket] get aria_port=" + this.portno
				+ " from " + connectedNode.getName() + "/ARIA_SOCKET_PORT");

		System.out.print("[CurrentorSocket] get aria_com_step_time=");
		if (params.getInteger(connectedNode.getName()
				+ "/ARIA_SOCKET_COM_STEP_TIME", -1) > 0) {
			this.com_step_time = params.getInteger(connectedNode.getName()
					+ "/ARIA_SOCKET_COM_STEP_TIME", -1);
		} else {
			this.com_step_time = 15;
		}
		System.out.println(this.com_step_time + " from "
				+ connectedNode.getName() + "/ARIA_SOCKET_COM_STEP_TIME");

		//this.mode = CurrentorSocketNode.NOP;
		this.last_request_time = System.currentTimeMillis();

		this.sensor_pub = new Publisher[CurrentorUtil.sensor_names.length];
		int id = 0;
		for (String name : CurrentorUtil.sensor_names) {
			this.sensor_pub[id++] = connectedNode.newPublisher(
					CurrentorSocketNode.nodename + "/sensor_array/" + name,
					std_msgs.Float32MultiArray._TYPE);
		}

		this.sensor_name_pub = connectedNode.newPublisher(
				CurrentorSocketNode.nodename + "/sensor_array/names",
				std_msgs.String._TYPE);
		this.currentor_socket_status = connectedNode
				.newPublisher(CurrentorSocketNode.nodename + "/status",
						std_msgs.String._TYPE);

		Subscriber<std_msgs.Float32MultiArray> trq_sub = connectedNode
				.newSubscriber(CurrentorSocketNode.nodename
						+ "/request/torque_vector",
						std_msgs.Float32MultiArray._TYPE);
		trq_sub.addMessageListener(
				new MessageListener<std_msgs.Float32MultiArray>() {
					@Override
					public void onNewMessage(std_msgs.Float32MultiArray arg) {
						synchronized (CurrentorSocketNode.this) {
							System.out.println(" -- torque command received "
									+ (System.currentTimeMillis() - CurrentorSocketNode.this.last_request_time)
									+ "ms");
							CurrentorSocketNode.this.vectorCommandEnqueue(
									CurrentorSocketNode.TRQ, arg.getData());
							CurrentorSocketNode.this.last_request_time = System
									.currentTimeMillis();
						}
					}
				}, 1);

		Subscriber<std_msgs.Float32MultiArray> whl_trq_sub = connectedNode
				.newSubscriber(CurrentorSocketNode.nodename
						+ "/request/wheel_torque_vector",
						std_msgs.Float32MultiArray._TYPE);
		whl_trq_sub.addMessageListener(
				new MessageListener<std_msgs.Float32MultiArray>() {
					@Override
					public void onNewMessage(std_msgs.Float32MultiArray arg) {
						synchronized (CurrentorSocketNode.this) {
							System.out.println(" -- torque command received "
									+ (System.currentTimeMillis() - CurrentorSocketNode.this.last_request_time)
									+ "ms");
							CurrentorSocketNode.this.vectorCommandEnqueue(
									CurrentorSocketNode.WHL, arg.getData());
							CurrentorSocketNode.this.last_request_time = System
									.currentTimeMillis();
						}
					}
				}, 1);

		Subscriber<std_msgs.Float32MultiArray> pos_sub = connectedNode
				.newSubscriber(CurrentorSocketNode.nodename
						+ "/request/position_vector",
						std_msgs.Float32MultiArray._TYPE);
		pos_sub.addMessageListener(
				new MessageListener<std_msgs.Float32MultiArray>() {
					@Override
					public void onNewMessage(std_msgs.Float32MultiArray arg) {
						synchronized (CurrentorSocketNode.this) {
							System.out.println(" -- position command received "
									+ (System.currentTimeMillis() - CurrentorSocketNode.this.last_request_time)
									+ "ms");
							CurrentorSocketNode.this.vectorCommandEnqueue(
									CurrentorSocketNode.POS, arg.getData());
							CurrentorSocketNode.this.last_request_time = System
									.currentTimeMillis();
						}
					}
				}, 1);

		Subscriber<std_msgs.Float32MultiArray> mod_sub = connectedNode
				.newSubscriber(CurrentorSocketNode.nodename
						+ "/request/mode_vector",
						std_msgs.Float32MultiArray._TYPE);
		mod_sub.addMessageListener(
				new MessageListener<std_msgs.Float32MultiArray>() {
					@Override
					public void onNewMessage(std_msgs.Float32MultiArray arg) {
						synchronized (CurrentorSocketNode.this) {
							System.out.println(" -- mode command received "
									+ (System.currentTimeMillis() - CurrentorSocketNode.this.last_request_time)
									+ "ms");
							CurrentorSocketNode.this.vectorCommandEnqueue(
									CurrentorSocketNode.MOD, arg.getData());
							CurrentorSocketNode.this.last_request_time = System
									.currentTimeMillis();
						}
					}
				}, 1);

		Subscriber<std_msgs.Float32MultiArray> tmax_sub = connectedNode
				.newSubscriber(CurrentorSocketNode.nodename
						+ "/request/torque_max_vector",
						std_msgs.Float32MultiArray._TYPE);
		tmax_sub.addMessageListener(
				new MessageListener<std_msgs.Float32MultiArray>() {
					@Override
					public void onNewMessage(std_msgs.Float32MultiArray arg) {
						synchronized (CurrentorSocketNode.this) {
							System.out.println(" -- tmax command received "
									+ (System.currentTimeMillis() - CurrentorSocketNode.this.last_request_time)
									+ "ms");
							CurrentorSocketNode.this.vectorCommandEnqueue(
									CurrentorSocketNode.TMAX, arg.getData());
							CurrentorSocketNode.this.last_request_time = System
									.currentTimeMillis();
						}
					}
				}, 1);

		Subscriber<std_msgs.Float32MultiArray> tmin_sub = connectedNode
				.newSubscriber(CurrentorSocketNode.nodename
						+ "/request/torque_min_vector",
						std_msgs.Float32MultiArray._TYPE);
		tmin_sub.addMessageListener(
				new MessageListener<std_msgs.Float32MultiArray>() {
					@Override
					public void onNewMessage(std_msgs.Float32MultiArray arg) {
						synchronized (CurrentorSocketNode.this) {
							System.out.println(" -- tmin command received "
									+ (System.currentTimeMillis() - CurrentorSocketNode.this.last_request_time)
									+ "ms");
							CurrentorSocketNode.this.vectorCommandEnqueue(
									CurrentorSocketNode.TMIN, arg.getData());
							CurrentorSocketNode.this.last_request_time = System
									.currentTimeMillis();
						}
					}
				}, 1);

		Subscriber<std_msgs.Float32MultiArray> pid_sub = connectedNode
				.newSubscriber(CurrentorSocketNode.nodename
						+ "/request/pid_vector",
						std_msgs.Float32MultiArray._TYPE);
		pid_sub.addMessageListener(
				new MessageListener<std_msgs.Float32MultiArray>() {
					@Override
					public void onNewMessage(std_msgs.Float32MultiArray arg) {
						synchronized (CurrentorSocketNode.this) {
							System.out.println(" -- pid command received "
									+ (System.currentTimeMillis() - CurrentorSocketNode.this.last_request_time)
									+ "ms");
							CurrentorSocketNode.this.vectorCommandEnqueue(
									CurrentorSocketNode.PID, arg.getData());
							CurrentorSocketNode.this.last_request_time = System
									.currentTimeMillis();
						}
					}
				}, 1);

		Subscriber<std_msgs.Float32MultiArray> ctv_sub = connectedNode
				.newSubscriber(CurrentorSocketNode.nodename
						+ "/request/ctv_vector",
						std_msgs.Float32MultiArray._TYPE);
		ctv_sub.addMessageListener(
				new MessageListener<std_msgs.Float32MultiArray>() {
					@Override
					public void onNewMessage(std_msgs.Float32MultiArray arg) {
						synchronized (CurrentorSocketNode.this) {
							System.out.println(" -- ctv command received "
									+ (System.currentTimeMillis() - CurrentorSocketNode.this.last_request_time)
									+ "ms");
							CurrentorSocketNode.this.vectorCommandEnqueue(
									CurrentorSocketNode.CTV, arg.getData());
							CurrentorSocketNode.this.last_request_time = System
									.currentTimeMillis();
						}
					}
				}, 1);

		Subscriber<std_msgs.Float32MultiArray> dtq_sub = connectedNode
				.newSubscriber(CurrentorSocketNode.nodename
						+ "/request/torque_combined_vector",
						std_msgs.Float32MultiArray._TYPE);
		dtq_sub.addMessageListener(
				new MessageListener<std_msgs.Float32MultiArray>() {
					@Override
					public void onNewMessage(std_msgs.Float32MultiArray arg) {
						synchronized (CurrentorSocketNode.this) {
							System.out.println(" -- dtorque command received "
									+ (System.currentTimeMillis() - CurrentorSocketNode.this.last_request_time)
									+ "ms");
							CurrentorSocketNode.this.vectorCommandEnqueue(
									CurrentorSocketNode.DTQ, arg.getData());
							CurrentorSocketNode.this.last_request_time = System
									.currentTimeMillis();
						}
					}
				}, 1);

		connectedNode.executeCancellableLoop(new CancellableLoop() {
			private long last_time = System.currentTimeMillis();
			private long step = CurrentorSocketNode.this.com_step_time;
			private String default_command = CurrentorUtil
					.encodeJsonCommand("getValues");

			// private std_msgs.String ros_res =
			// CurrentorSocketNode.this.response_pub.newMessage();
			@Override
			protected void loop() throws InterruptedException {
				synchronized (CurrentorSocketNode.this) {
					if (CurrentorSocketNode.this.connected
							|| CurrentorSocketNode.this.openConnection(
									CurrentorSocketNode.this.hostname,
									CurrentorSocketNode.this.portno)) {
						String res;
						String command;
						int mode = CurrentorSocketNode.NOP;
						float[] requested_data = null;
						CurrentorVectorCommand data = CurrentorSocketNode.this.vectorCommandPop();
						if ( data != null ){
							mode = data.mode;
							requested_data = data.requested_data;
						}
						for ( CurrentorVectorCommand d : CurrentorSocketNode.this.vectorCommand ){
						    d.dump();
						}
						switch (mode) {
						case CurrentorSocketNode.TRQ:
							command = CurrentorUtil.encodeJsonCommand(
									"setTorques",
									requested_data);
							if (command == null) {
								command = this.default_command;
								System.out
										.println(" -- Torque command rejected/");
							}
							res = CurrentorSocketNode.this
									.postConnection(command);
							break;
						case CurrentorSocketNode.DTQ:
							command = CurrentorUtil.encodeJsonCommand(
									"setTorquesCombined",
									requested_data);
							if (command == null) {
								command = this.default_command;
								System.out
										.println(" -- DTorque command rejected/");
							}
							res = CurrentorSocketNode.this
									.postConnection(command);
							break;
						case CurrentorSocketNode.WHL:
							command = CurrentorUtil.encodeJsonCommand(
									"setWheelTorques",
									requested_data, 2);
							if (command == null) {
								command = this.default_command;
								System.out
										.println(" -- Wheel torque command rejected/");
							}
							res = CurrentorSocketNode.this
									.postConnection(command);
							break;
						case CurrentorSocketNode.POS:
							command = CurrentorUtil.encodeJsonCommand(
									"setPositions",
									requested_data);
							if (command == null) {
								command = this.default_command;
								System.out
										.println(" -- Position command rejected/");
							}
							res = CurrentorSocketNode.this
									.postConnection(command);
							break;
						case CurrentorSocketNode.MOD:
							command = CurrentorUtil.encodeJsonCommand(
									"setControlModes",
									requested_data);
							if (command == null) {
								command = this.default_command;
								System.out
										.println(" -- Mode command rejected/");
							}
							res = CurrentorSocketNode.this
									.postConnection(command);
							break;
						case CurrentorSocketNode.TMAX:
							command = CurrentorUtil.encodeJsonCommand(
									"setMaxLimit",
									requested_data);
							if (command == null) {
								command = this.default_command;
								System.out
										.println(" -- TMAX command rejected/");
							}
							res = CurrentorSocketNode.this
									.postConnection(command);
							break;
						case CurrentorSocketNode.TMIN:
							command = CurrentorUtil.encodeJsonCommand(
									"setMinLimit",
									requested_data);
							if (command == null) {
								command = this.default_command;
								System.out
										.println(" -- TMIN command rejected/");
							}
							res = CurrentorSocketNode.this
									.postConnection(command);
							break;
						case CurrentorSocketNode.PID:
							command = CurrentorUtil.encodeJsonCommand(
									"setPIDGain",
									requested_data,
									3 * CurrentorUtil.joint_cnt);
							if (command == null) {
								command = this.default_command;
								System.out.println(" -- pid command rejected/");
							}
							res = CurrentorSocketNode.this
									.postConnection(command);
							break;
						case CurrentorSocketNode.CTV:
							command = CurrentorUtil.encodeJsonCommand(
									"setCTVGain",
									requested_data);
							if (command == null) {
								command = this.default_command;
								System.out.println(" -- ct command rejected/");
							}
							res = CurrentorSocketNode.this
									.postConnection(command);
							break;
						case CurrentorSocketNode.NOP:
						default:
							res = CurrentorSocketNode.this
									.postConnection(this.default_command);
							break;
						}
						//
						if (CurrentorUtil.decodeJsonCommand(res)) {
							std_msgs.String ros_res = CurrentorSocketNode.this.currentor_socket_status
									.newMessage();
							ros_res.setData(res);
							CurrentorSocketNode.this.currentor_socket_status
									.publish(ros_res);
						} else {
						    CurrentorSocketNode.this.vectorCommandEnqueue(
									mode, requested_data);
						}
						CurrentorSocketNode.this.publishSensors();
					} else {
						System.out.println("connection refused: "
								+ CurrentorSocketNode.this.hostname + ":"
								+ CurrentorSocketNode.this.portno);
						// std_msgs.String ros_res =
						// CurrentorSocketNode.this.response_pub.newMessage();
						// ros_res.setData("connection refused: "
						// + CurrentorSocketNode.this.hostname + ":"
						// + CurrentorSocketNode.this.portno);
						// CurrentorSocketNode.this.response_pub.publish(ros_res);
					}
				}
				long now = System.currentTimeMillis();
				// System.out.println(" --- time: " + (now - this.last_time) +
				// "[ms]") ;
				if (step > now - this.last_time) {
					Thread.sleep(step - (now - this.last_time));
					// System.out.println(" --- sleep: " + (step- ( now -
					// this.last_time )) + "[ms]") ;
				} else {
					System.out.println(" --- overslept...");
				}
				this.last_time = System.currentTimeMillis();
			}

		});

		// this.thread = new Thread(this);
		// this.thread.start();
	}

	private void publishSensors() {
		for (int i = 0; i < CurrentorUtil.sensor_names.length; i++) {
			std_msgs.Float32MultiArray mes = this.sensor_pub[i].newMessage();
			mes.setData(CurrentorUtil.sensor_values[i]);
			this.sensor_pub[i].publish(mes);
		}
	}

	static class CurrentorUtil {
		static String[] sensor_names = new String[] { "position", "velocity",
				"temperature", "torque", "mode", "debug", "imu" };
		static int joint_cnt = 30;

		static byte[] byte_buf = new byte[4 * (CurrentorUtil.joint_cnt
				* (CurrentorUtil.sensor_names.length - 1) + 6)];
		static float[][] sensor_values = new float[CurrentorUtil.sensor_names.length][];
		static {
			for (int i = 0; i < sensor_names.length - 1; i++) {
				sensor_values[i] = new float[CurrentorUtil.joint_cnt];
			}
			sensor_values[sensor_names.length - 1] = new float[6];
		}

		public static boolean decodeSensorValueString(String in) {
			if (in.length() != CurrentorUtil.byte_buf.length * 2) {
				System.out
						.println("[decodeSensorValueString] invalid input string "
								+ in
								+ " length "
								+ in.length()
								+ " vs "
								+ CurrentorUtil.byte_buf.length * 2);
				return false;
				// in = in.substring(0,CurrentorUtil.byte_buf.length*2);
			}
			FloatString.xstring2barray(in, CurrentorUtil.byte_buf);
			FloatString.barray2farray(CurrentorUtil.byte_buf,
					CurrentorUtil.sensor_values, sensor_names.length - 1);
			return true;
		}

		public static String encodeFloatString(float[] in, int array_size) {
			if (in.length != array_size) {
				System.out
						.println("[encodeFloatString] invalid input vector length "
								+ in.length + " vs " + array_size);
				return null;
			}
			byte[] buf = FloatString.farray2barray(in);
			return FloatString.barray2xstring(buf);
		}

		public static String encodeFloatString(float[] in) {
			return CurrentorUtil.encodeFloatString(in, CurrentorUtil.joint_cnt);
		}

		public static String encodeJsonCommand(String func) {
			return String.format(
					"{\"method\":\"%s\",\"params\":[0],\"id\":\"1\"}", func);
		}

		public static String encodeJsonCommand(String func, float[] array) {
			return CurrentorUtil.encodeJsonCommand(func, array,
					CurrentorUtil.joint_cnt);
		}

		public static String encodeJsonCommand(String func, float[] array,
				int array_size) {
			String param = CurrentorUtil.encodeFloatString(array, array_size);
			if (param != null) {
				return String.format(
						"{\"method\":\"%s\",\"params\":[%s],\"id\":\"1\"}",
						func, param);
			} else {
				return null;
			}
		}

		public static boolean decodeJsonCommand(String json) {
			String[] split = json.split(":|,");
			int i = 0;
			for (String str : split) {
				i++;
				if (str.contains("result")) {
					break;
				}
			}
			if (i >= split.length || (json = split[i].trim()).length() < 2) {
				System.out.println("[decodeJsonCommand] result missing in "
						+ json);
				return false;
			}
			json = json.substring(1, json.length() - 1);
			return CurrentorUtil.decodeSensorValueString(json);
		}

	}
}
