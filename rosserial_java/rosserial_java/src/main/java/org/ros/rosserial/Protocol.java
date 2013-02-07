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

package org.ros.rosserial;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.HashMap;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

import org.ros.message.Message;
import org.ros.message.MessageDeserializer;
import org.ros.message.rosserial_msgs.Log;
import org.ros.message.rosserial_msgs.TopicInfo;
import org.ros.service.rosserial_msgs.RequestParam;
import org.ros.node.Node;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import org.ros.rosserial.TopicRegistrationListener;

/**
 * Protocol handler for rosserial.
 * 
 * @author Adam Stambler
 */
public class Protocol {

	// SPECIAL IDS
	// All IDS greater than 100 are Publishers/Subscribers
	static final int TOPIC_PUBLISHERS = 0;
	static final int TOPIC_SUBSCRIBERS = 1;
	static final int TOPIC_TIME = 10;
	
	public static final byte[] NEGOTIATE_TOPICS_REQUEST = { (byte) 0, (byte) 0, (byte) 0, (byte) 0 };

	/**
	 * Node hosting the subscribers and publishers.
	 */
	private Node node;

	/**
	 * Map of IDs being sent down the channel to the topics they represent.
	 */
	private Map<Integer, TopicInfo> id_to_topic = new HashMap<Integer, TopicInfo>();

	/**
	 * Map of topic names to the IDs being sent down the channel for them.
	 */
	private Map<String, Integer> topic_to_id = new HashMap<String, Integer>();

	/**
	 * Topic ID to publisher.
	 */
	private Map<Integer, Publisher> publishers = new HashMap<Integer, Publisher>();
	
	/**
	 * Topic ID to subscriber.
	 */
	private Map<Integer, Subscriber> subscribers = new HashMap<Integer, Subscriber>();
	
	/**
	 * Topic ID to message deserializer for the associated topic message.
	 */
	private Map<Integer, MessageDeserializer> msg_deserializers = new HashMap<Integer, MessageDeserializer>();

	/**
	 * Listener for new publisher registrations.
	 */
	private TopicRegistrationListener newPubListener;

	/**
	 * Listener for new subscriber registrations.
	 */
	private TopicRegistrationListener newSubListener;

	/**
	 * Handles wire communication to the remote endpoint.
	 */
	private PacketHandler packetHandler;

	public Protocol(Node nh, PacketHandler handler) {
		this.node = nh;
		this.packetHandler = handler;
		this.paramT = nh.newParameterTree();
	}

	/**
	 * Set a new topic registration listener for publications.
	 * 
	 * @param listener
	 */
	public void setOnNewPublication(TopicRegistrationListener listener) {
		newPubListener = listener;
	}

	/**
	 * Set a new topic registration listener for subscriptions.
	 * 
	 * @param listener
	 */
	public void setOnNewSubcription(TopicRegistrationListener listener) {
		newSubListener = listener;
	}

	/**
	 * Ask the remote endpoint for any topics it wants to publish or subscribe to.
	 */
	public void negotiateTopics() {
		packetHandler.send(NEGOTIATE_TOPICS_REQUEST);
	}

	/**
	 * Construct a valid protocol message. This take the id and m, serializes
	 * them and return the raw bytes to be sent
	 */
	public byte[] constructMessage(int id, org.ros.message.Message m) {
		int l = m.serializationLength();
		byte[] data = new byte[l + 4];
		ByteBuffer buff = ByteBuffer.wrap(data, 4, l);

		data[0] = (byte) id;
		data[1] = (byte) (id >> 8);
		data[2] = (byte) l;
		data[3] = (byte) (l >> 8);

		m.serialize(buff, 0);

		return data;
	}

	/**
	 * ! Registers a topic being transmitted over the serial port /param topic-
	 * The topic info msg describing the topic /param is_publisher - is the
	 * device on the other end of the serial line publishing
	 */
	private void addTopic(TopicInfo topic, boolean is_publisher) {
		String name = topic.topic_name;
		String type = topic.message_type;
		Integer id = topic.topic_id;
		// check if its already registered
		if (id_to_topic.containsKey(id)) {
			if (id_to_topic.get(id).topic_name.equals(name))
				return;
		}
		try {
			msg_deserializers.put(topic.topic_id,
					node.getMessageSerializationFactory()
							.newMessageDeserializer(type));
			topic_to_id.put(topic.topic_name, topic.topic_id);
			id_to_topic.put(id, topic);

			if (is_publisher) {
				Publisher pub = node.newPublisher(name, type);
				publishers.put(id, pub);
				node.getLog().info(
						"Adding Publisher " + name + " of type " + type);
				if (newPubListener != null)
					newPubListener.onNewTopic(topic);
			} else {
				Subscriber sub = node.newSubscriber(name, type,
						new MessageListenerForwarding(id, this));
				subscribers.put(id, sub);
				node.getLog().info(
						"Adding Subscriber " + name + " of type " + type);
				if (newSubListener != null)
					newSubListener.onNewTopic(topic);
			}
		} catch (Exception e) {
			node.getLog().error("Exception while adding topic", e);
		}
	}

	public TopicInfo[] getSubscriptions() {
		TopicInfo[] topics = new TopicInfo[subscribers.size()];

		int i = 0;
		for (Integer id : subscribers.keySet()) {
			topics[i++] = id_to_topic.get(id);
		}
		return topics;
	}

	public TopicInfo[] getPublications() {
		TopicInfo[] topics = new TopicInfo[publishers.size()];

		int i = 0;
		for (Integer id : publishers.keySet()) {
			topics[i++] = id_to_topic.get(id);
		}
		return topics;
	}

	/**
	 * This timer handles monitoring handles monitoring the connection
	 * to the device;
	 */
	private Timer connection_timer =  new Timer();
	static final int CONNECTION_TIMOUT_PERIOD = 10000;
	TimerTask timer_cb = new TimerTask() {
		
		@Override
		public void run() {
			// TODO Auto-generated method stub
			if (sync_requested){
				connected = true;
				sync_requested = false;
				
			}
			else{
				node.getLog().info("Connection to client lost. Topic negotiation requested");
				connected = false;
				negotiateTopics();
			}
		}
	};
	
	private boolean sync_requested=false;
	private boolean connected =false;
	
	public void start(){
		connection_timer.scheduleAtFixedRate(timer_cb, CONNECTION_TIMOUT_PERIOD,  CONNECTION_TIMOUT_PERIOD);
	}
	
	/**
	 * Parse a packet from the remote endpoint.
	 * 
	 * @param topic_id
	 *            ID of the message topic.
	 * @param msg_data
	 *            The data for the message.
	 * 
	 * @return
	 */
	public boolean parsePacket(int topic_id, byte[] msg_data) {

		switch (topic_id) {
		case TopicInfo.ID_PUBLISHER:
			TopicInfo pm = new TopicInfo();
			pm.deserialize(msg_data);
			addTopic(pm, true);
			connected = true;
			break;

		case TopicInfo.ID_SUBSCRIBER:
			TopicInfo sm = new TopicInfo();
			sm.deserialize(msg_data);
			addTopic(sm, false);
			connected =true;
			break;

		case TopicInfo.ID_SERVICE_SERVER:
			break;
		case TopicInfo.ID_SERVICE_CLIENT:
			break;
		case TopicInfo.ID_PARAMETER_REQUEST:
			handleParameterRequest(msg_data);
			break;
		case TopicInfo.ID_LOG:
			handleLogging(msg_data);
			break;
		case TopicInfo.ID_TIME:
			sync_requested = true;
			org.ros.message.Time t = node.getCurrentTime();
			org.ros.message.std_msgs.Time t_msg = new org.ros.message.std_msgs.Time();
			t_msg.data = t;
			packetHandler.send(constructMessage(TOPIC_TIME, t_msg));
			break;

		default:
			MessageDeserializer c = msg_deserializers.get(topic_id);
			if (c != null) {
				
			    ByteBuffer bb = ByteBuffer.wrap(msg_data);
			    Message msg = (Message) c.deserialize(bb.asReadOnlyBuffer().order(ByteOrder.LITTLE_ENDIAN));
				publishers.get(topic_id).publish(msg);
			} else {
				node.getLog().info(
						"Trying to publish to unregistered ID #" + topic_id);

				// Try to negotiate topics then
				negotiateTopics();
			}
			break;
		}

		return false;
	}

	/**
	 * Handle Logging takes the log message from rosserial and rebroadcasts it 
	 * via rosout at the appropriate logging level
	 * @param msg_data
	 */
	private void  handleLogging(byte[] msg_data){
		Log log_msg = new Log();
		log_msg.deserialize(msg_data);
		switch(log_msg.level){
		case Log.DEBUG:
			node.getLog().debug(log_msg.msg);
			break;
		case Log.INFO:
			node.getLog().info(log_msg.msg);
			break;
		case Log.WARN:
			node.getLog().warn(log_msg.msg);
			break;
		case Log.ERROR:
			node.getLog().error(log_msg.msg);
			break;
		case Log.FATAL:
			node.getLog().fatal(log_msg.msg);
			break;
		}
	}
	
	
	ParameterTree paramT ;
	private void handleParameterRequest(byte[] msg_data){
		RequestParam rp = new RequestParam();
		RequestParam.Request req = rp.createRequest();
		req.deserialize(msg_data);
		
		RequestParam.Response resp = rp.createResponse();		

	}
	
	/**
	 * Forwards messages via a subscriber callback for a subscriber topic to the
	 * packet handler which communicates with the remote endpoint.
	 * 
	 * @author Adam Stambler
	 */
	private static class MessageListenerForwarding<MessageType> implements
			org.ros.message.MessageListener<MessageType> {
		/**
		 * The protocol handler handling the communication for the topic.
		 */
		private Protocol protocol;

		/**
		 * The topic ID for this listener.
		 */
		private int id;

		public MessageListenerForwarding(int topic_id, Protocol p) {
			protocol = p;
			id = topic_id;
		}

		@Override
		public void onNewMessage(MessageType t) {
			byte[] data = protocol.constructMessage(id, (Message) t);
			protocol.packetHandler.send(data);
		}
	}

	/**
	 * Handles communication to the remote endpoint.
	 * 
	 * @author Adam Stambler
	 */
	public interface PacketHandler {
		/**
		 * Send data to the remote endpoint.
		 * 
		 * @param data
		 *            The data to send.
		 */
		void send(byte[] data);
	}
}
