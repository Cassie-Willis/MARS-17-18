package mars;

import java.io.*;
import java.net.*;

public class NetworkDaemon {
	private Socket connection;
	// used for connecting to robot
	private String robotAddress;
	// used for connecting to test server, if in debug mode
	private String debugServerName;
	private String connectAddress;
	// default port number; 5565
	private int port;
	private volatile boolean connected;

	public NetworkDaemon(String robotAddress, int port) {
		this.robotAddress = robotAddress;
		debugServerName = "localhost";
		this.port = port;
		connected = false;
	}

	// connects to robot by setting up a socket connection
	public void connect(boolean isDebug) throws IOException {
		// if in debug mode, connect to test server, otherwise attempt to
		// connect to robot
		if (isDebug) {
			connectAddress = debugServerName;
		} else {
			connectAddress = robotAddress;
		}
		// attempt to create a socket connection with the specified connect
		// address,
		// times out after one second if no connection is found
			connection = new Socket();
			connection.connect(new InetSocketAddress(connectAddress, port), 1000);
			connected = true;
	}

	// sends byte array of joystick to robot
	public boolean send(byte[] byteArr) throws IOException {
		// makes sure socket is connected
			OutputStream out = connection.getOutputStream();
			out.write(byteArr, 0, byteArr.length);
			return true;

	}

	// receives byte array from test server/robot
	public byte[] receive() throws IOException {
		// expanding byte array
		try {
			ByteArrayOutputStream baos = new ByteArrayOutputStream();
			byte[] received = new byte[3];
			if (connected) {
				InputStream in = connection.getInputStream();
				int numBytes = in.read(received);
				int i = 0;
				while (baos.size() < numBytes) {
					baos.write(received, i, 1);
					i++;
				}
			}
			return baos.toByteArray();
		} catch (IOException e) {
			connected = false;
			throw e;
		}
	}

	public void setConnected(boolean b) {
		connected = b;
	}

	// determines if is connnected
	public boolean isConnected() {
		boolean connected;
		try {
			connected = connection.getInetAddress().isReachable(5000);
			connected = true;
		} catch (IOException e) {
			connected = false;
			e.printStackTrace();
		}

		return connected;
	}

	// attempts to close the socket connection
	public void disconnect() {
		try {
			connection.close();
			connected = false;
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

}