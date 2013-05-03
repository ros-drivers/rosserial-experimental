package org.ros.rosserial;

import android.app.NotificationManager;
import android.app.Service;
import android.content.Intent;
import android.media.ExifInterface;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;
import android.widget.TextView;
import android.widget.Toast;

import org.ros.node.Node;
import org.ros.RosUtils;
import org.ros.message.rosserial_msgs.TopicInfo;
import org.ros.rosserial.ROSSerialADK;

import com.android.future.usb.UsbManager;

public class ROSSerialADKService extends Service {
	static final String TAG = "ROSSerialSDKService";
	
	/**
	 * ROS node name
	 */
	private String node_name;
	
	/**
	 * ROS node handle
	 */
	Node nh;
	
	/**
	 * Reference to ADK
	 */
	ROSSerialADK adk = null;
	
	/**
	 * IBinder to connect to ADK Service
	 */
    private  IBinder mBinder;
    
    /**
     * Binder class for connecting to ADK service
     * Assumes that everything is in the same processes
     * and returns a copy of the ROSSerialADK
     * @author Adam Stambler
     *
     */
    public class LocalBinder extends Binder {
        ROSSerialADKService service;
        public LocalBinder(ROSSerialADKService s) {
        	service = s;
		}
    	ROSSerialADK getADK() {
            // Return this instance of LocalService so clients can call public methods
            return adk;
        }
        ROSSerialADKService getService(){
        	return service;}
    }


	@Override
	public IBinder onBind(Intent intent) {
		return mBinder;
	}
	

	/**
	 * Starts up Service
	 * Creates ROS node and registers with master
	 * 
	 */
	@Override
	public int onStartCommand(Intent intent, int flags, int startId) {

		String master_uri = intent.getExtras().getString("ROS_MASTER_URI");
		if (master_uri == null) master_uri = "http://localhost:11311";
		
		node_name = intent.getExtras().getString("name");
		if (node_name ==null) node_name = "ROSSerialADK";

		Log.v(TAG, "Starting ROSSerialADKService as node '" + node_name+"' with master '" + master_uri);

		try{
			nh = RosUtils.createExternalMaster(node_name, master_uri);
			adk = new ROSSerialADK(this, nh);	
		}
		catch(Exception e){
			// TODO Properly handle failure to connect to the ROS Master

			e.printStackTrace();
			System.exit(-1);
		}
		if (!adk.open()){
			Log.v(TAG, "No ADK connected");
		}
		
		mBinder = new LocalBinder(this);
		
		Toast t = Toast.makeText(this, node_name + " started.", Toast.LENGTH_SHORT);
		t.show();
		
		return START_STICKY;
	}

	@Override
	public void onDestroy() {
		if (adk!=null) this.adk.shutdown();
		if (nh != null) this.nh.shutdown();
		Toast t = Toast.makeText(this, node_name + " stopped.", Toast.LENGTH_SHORT);
		t.show();
	
		super.onDestroy();
	}

	public ROSSerialADKService(){
		super();
	}
	
}
