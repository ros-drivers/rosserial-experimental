package org.ros.rosserial;

import org.ros.message.rosserial_msgs.TopicInfo;


/**
 * Listener for notification of new subscriptions or publications.
 *
 * @author Adam Stambler
 */
public interface TopicRegistrationListener {
	/**
	 * A new topic has come in.
	 * 
	 * @param t Information about the new topic.
	 */
	void onNewTopic(TopicInfo t);
}