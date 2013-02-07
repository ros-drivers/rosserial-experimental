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
import org.ros.rosserial.*;


import java.util.*;
import gnu.io.*;

import org.ros.node.Node;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import java.io.FileOutputStream;

public class SerialNode implements NodeMain {

	  private Node node;


	public static SerialPort createSerialPort(String portName){
		Enumeration portIdentifiers = CommPortIdentifier.getPortIdentifiers();
		CommPortIdentifier portId = null;  // will be set if port found
		
		while (portIdentifiers.hasMoreElements())
		{
		    CommPortIdentifier pid = (CommPortIdentifier) portIdentifiers.nextElement();
		    if(pid.getPortType() == CommPortIdentifier.PORT_SERIAL &&
		       pid.getName().equals(portName)) 
		    {
		        portId = pid;
		        break;
		    }
		}
		if(portId == null)
		{
		    System.err.println("Could not find serial port " + portName);
		    System.exit(1);
		}
		
		SerialPort port = null;
		try {
		    port = (SerialPort) portId.open(
		        "rosserial_java", // Name of the application asking for the port 
		        10000   	// Wait max. 10 sec. to acquire port
		    );
		} catch(PortInUseException e) {
		    System.err.println("Port already in use: " + e);
		    System.exit(1);
		}
	
		return port;
	}

	  @Override
	  public void main(NodeConfiguration configuration) {
	    try {
		  System.out.println("Starting RosSerial node");
		  DefaultNodeFactory nodeFactory = new DefaultNodeFactory();
	      node = nodeFactory.newNode("rosserial_node", configuration);

			String portName = "/dev/ttyUSB0";
			SerialPort port= createSerialPort(portName);
			
		
			port.setSerialPortParams(57600,    
					SerialPort.DATABITS_8,
					SerialPort.STOPBITS_1,
					SerialPort.PARITY_NONE);
			
			//clear the librxtx serial port.  The first request for topics
			//is normally missed if this is not done.
			for(int i=0; i<50; i++)port.getOutputStream().write(0);
			Thread.sleep(1000);
			for(int i=0; i<50; i++)port.getOutputStream().write(0);


			ROSSerial rs = new ROSSerial(node, port.getInputStream(), port.getOutputStream());
			System.out.println("Now running RosSerial Node connected to " + portName);
			rs.run();	
			
		}catch (Exception e) {
	      if (node != null) {
		        e.printStackTrace();
	      } else {
	        e.printStackTrace();
	      }
	    }
	  }

	  @Override
	  public void shutdown() {
	    node.shutdown();
	    node = null;
	  }

	}
