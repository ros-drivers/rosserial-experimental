package org.ros;

import java.net.InetAddress;

import org.ros.node.Node;
import org.ros.address.InetAddressFactory;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.NodeConfiguration;

public class RosUtils {

	public static Node createExternalMaster(String node_name, String masterURI){
	      java.net.URI muri =java.net.URI.create( masterURI);
          
          InetAddress host = InetAddressFactory.newNonLoopback();
          NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host.getHostName(), muri);

         DefaultNodeFactory factory = new DefaultNodeFactory();
         return  factory.newNode(node_name, nodeConfiguration);
	}
}
