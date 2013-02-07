// Software License Agreement (BSD License)
//
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Willow Garage, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// Author: Adam Stambler  <adasta@gmail.com>
package org.ros.rosserial;

import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Timer;

import org.ros.message.rosserial_msgs.TopicInfo;
import org.ros.node.Node;

/**
 * The host computer endpoint for a rosserial connection.
 * 
 * @author Adam Stambler
 */
public class ROSSerial implements Runnable  {
	/**
	 * Flags for marking beginning of packet transmission
	 */
	public static final byte[] FLAGS = { (byte) 0xff, (byte) 0xff };

	/**
	 * Maximum size for the incomming message data in bytes
	 * Same as Message out buffer size in rosserial_arduino
	 */
	private static final int MAX_MSG_DATA_SIZE =512;
	
	/**
	 * Output stream for the serial line used for communication.
	 */
	private OutputStream ostream;

	/**
	 * Input stream for the serial line used for communication.
	 */
	private InputStream istream;

	/**
	 * The node which is hosting the publishers and subscribers.
	 */
	private Node node;

	/**
	 * Protocol handler being used for this connection.
	 */
	private Protocol protocol;
	
	/**
	 * Set a new topic registration listener for publications.
	 * 
	 * @param listener
	 */
	public void setOnNewPublication(TopicRegistrationListener listener) {
		protocol.setOnNewPublication(listener);
	}

	/**
	 * Set a new topic registration listener for subscriptions.
	 * 
	 * @param listener
	 */
	public void setOnNewSubcription(TopicRegistrationListener listener) {
		protocol.setOnNewSubcription(listener);
	} 
	

	public TopicInfo[] getSubscriptions() {
		return protocol.getSubscriptions();
	}

	public TopicInfo[] getPublications() {
		return protocol.getPublications();
	}
	
	/**
	 * True if this endpoint is running, false otherwise.
	 */
	private boolean running = false;

	// parsing state machine variables/enumes
	private enum PACKET_STATE {
		FLAGA, FLAGB, HEADER, DATA, CHECKSUM
	};

	private PACKET_STATE packet_state;
	private byte[] header = new byte[4];
	private byte[] data = new byte[MAX_MSG_DATA_SIZE];
	private int data_len = 0;
	private int byte_index = 0;;

	/**
	 * Packet handler for writing to the other endpoint.
	 */
	Protocol.PacketHandler sendHandler = new Protocol.PacketHandler() {

		@Override
		public void send(byte[] data) {
			// calculate the checksum
			int chk = 0;
			for (int i = 0; i < data.length; i++)
				chk += 0xff & data[i];
			chk = 255 - chk % 256;

			try {
				ostream.write(FLAGS);
				ostream.write(data);
				ostream.write((byte) chk);
			} catch (IOException e) {
				System.out.println("Exception sending :"
						+ BinaryUtils.byteArrayToHexString(data));
				e.printStackTrace();
			}
		}
	};

	public ROSSerial(Node nh, InputStream input, OutputStream output) {
		ostream = output;
		istream = input;
		node = nh;

		protocol = new Protocol(node, sendHandler);

	}

	/**
	 * Shut this endpoint down.
	 */
	public void shutdown() {
		running = false;
	}

	/**
	 * This timer watches when a packet starts.  If the packet
	 * does not complete itself within 30 milliseconds
	 * the message is thrown away.
	 */
	Timer packet_timeout_timer;
	
	/**
	 * Start running the endpoint.
	 */
	public void run() {
		
		protocol.negotiateTopics();
		protocol.start();
		
		resetPacket();

		running = true;

		// TODO
		// there should be a node.isOk() or something
		// similar so that it stops when ros is gone
		// but node.isOk() does not work, its never true...
		byte[] buffer = new byte[500];
		while (running) {
			try {
				int ret = istream.read(buffer);
				for(int i=0; i<ret;i++) {
					handleByte((byte) (0xff & buffer[i]));
				}
			} catch (IOException e) {
				node.getLog().error("Unable to read input stream", e);
				System.out.println("Unable to read input stream");

				if (e.toString().equals("java.io.IOException: No such device")) {
					node.getLog()
							.error("Total IO Failure.  Now exiting ROSSerial iothread.");
					break;
				}
				resetPacket();
			} catch (Exception e) {
				node.getLog().error("Unable to read input stream", e);
			}
			try {
				//Sleep prevents continuous polling of istream.
				//continuous polling kills an inputstream on android
				Thread.sleep(10);
			} catch (InterruptedException e) {
				e.printStackTrace();
				break;
			}
		}
		node.getLog().info("Finished ROSSerial IO Thread");		
	}

	/*
	 * ! reset parsing statemachine
	 */
	private void resetPacket() {
		byte_index = 0;
		data_len = 0;
		packet_state = PACKET_STATE.FLAGA;
	}

	/*
	 * ! handle byte takes an input byte and feeds it into the parsing
	 * statemachine /param b input byte /return true or falls depending on if
	 * the byte was successfully parsed
	 */
	private boolean handleByte(byte b) {
		switch (packet_state) {
		case FLAGA:
			if (b == (byte) 0xff)
				packet_state = PACKET_STATE.FLAGB;
			break;
		case FLAGB:
			if (b == (byte) 0xff) {
				packet_state = PACKET_STATE.HEADER;
			} else {
				resetPacket();
				return false;
			}
			break;
		case HEADER:
			header[byte_index] = b;
			byte_index++;
			if (byte_index == 4) {
				int len = (int) (header[3] << 8) | (int) (header[2]);
				data_len = len; // add in the header length
				byte_index = 0;
				packet_state = PACKET_STATE.DATA;
			}
			break;
		case DATA:
			data[byte_index] = b;
			byte_index++;
			if (byte_index == data_len) {
				packet_state = PACKET_STATE.CHECKSUM;
			}
			break;
		case CHECKSUM:
			int chk = (int) (0xff & b);
			for (int i = 0; i < 4; i++)
				chk += (int) (0xff & header[i]);
			for (int i = 0; i < data_len; i++) {
				chk += (int) (0xff & data[i]);
			}
			if (chk % 256 != 255) {
				resetPacket();
				System.out.println("Checksum failed!");
				return false;
			} else {
				int topic_id = (int) header[0] | (int) (header[1]) << 8;
				resetPacket();
				protocol.parsePacket(topic_id, data);
			}
			break;
		}
		return true;
	}

}
