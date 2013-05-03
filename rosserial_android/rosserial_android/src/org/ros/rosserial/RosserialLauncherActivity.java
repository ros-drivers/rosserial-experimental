package org.ros.rosserial;

import org.ros.message.rosserial_msgs.TopicInfo;
import org.ros.rosserial.ROSSerialADK;
import org.ros.rosserial.ROSSerialADKService.LocalBinder;

import android.app.Activity;
import android.app.ActivityManager;
import android.app.ActivityManager.RunningServiceInfo;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.Bundle;
import android.os.IBinder;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

public class RosserialLauncherActivity extends Activity {

	static final String TAG = "RosserialLauncherActivity";
	
	ROSSerialADK adk = null;
	Context mContext = null;
	LocalBinder mBinder= null;
	
	static private class TextViewHandler implements Runnable{
		TextView view;
		String msg;
		TextViewHandler(TextView v, String text){
			view = v;
			msg = text;
		}
		@Override
		public void run() {
			view.setText(msg);
		}
	}

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.launcher);
	
		mContext = this;
		
		Button b = (Button) findViewById(R.id.StartButton);
		b.setOnClickListener(start_click_listener);
		
		b = (Button) findViewById(R.id.StopButton);
		b.setOnClickListener(new OnClickListener() {
			
			@Override
			public void onClick(View arg0) {

				Log.v(TAG, "Trying to stop service");
				try{
					unbindService(mConnection);
					stopService(new Intent(mContext, ROSSerialADKService.class));
				}
				catch(IllegalArgumentException e){
					Toast t = Toast.makeText(mContext, "All instances of ROSSeriakADK are stopped.", Toast.LENGTH_SHORT);
					t.show();
				}
			}
		});
		
	}

	
	@Override
	protected void onResume() {

		super.onResume();
		if (isADKRunning()){
			//bind to it
			
			Intent i =  new Intent(mContext, ROSSerialADKService.class);
			try{
				bindService(i, mConnection, 0);
			}
			catch(Exception e){
				e.printStackTrace();
			}
		}
	}


	OnClickListener start_click_listener = new OnClickListener() {
		
		@Override
		public void onClick(View v) {
			TextView txt_MasterURi = (TextView) findViewById(R.id.MasterURITxt);
			TextView txt_NodeName =  (TextView) findViewById(R.id.NodeNameTxt);
			
			Intent i =  new Intent(mContext, ROSSerialADKService.class);

			i.putExtra("ROS_MASTER_URI", "http://"+txt_MasterURi.getText());
			i.putExtra("node name", txt_NodeName.getText());
			
			mContext.startService(i);
			bindService(i, mConnection, 0);
			Log.v(TAG,"Started sdk service");
		}
	};
	
	private boolean isADKRunning() {
	    ActivityManager manager = (ActivityManager) getSystemService(ACTIVITY_SERVICE);
	    for (RunningServiceInfo service : manager.getRunningServices(Integer.MAX_VALUE)) {
	        if ("org.ros.rosserial.ROSSerialADKService".equals(service.service.getClassName())) {
	            return true;
	        }
	    }
	    return false;
	}
	
	void setADKConnected(){
		TextView v = (TextView) findViewById(R.id.ConncectionStatusView);
		v.setText("ADK Is connected");
	}
	
	static void displayTopics(TextView v, String type, TopicInfo[] infos){
		int l =type.length();
		for(TopicInfo info : infos){
			l+= info.topic_name.length();
			l+= info.message_type.length();
		}
		StringBuilder msg = new StringBuilder(l+infos.length*5);
		msg.append(type);
		for(TopicInfo info : infos){
			msg.append(" : \n");

			msg.append("  ");
			msg.append(info.topic_name);
			msg.append(" - ");
			msg.append(info.message_type);
		}
		
		v.post(new TextViewHandler(v, msg.toString()) );					
		
	}
	
	private ServiceConnection mConnection = new ServiceConnection() {
		
		@Override
		public void onServiceDisconnected(ComponentName name) {
			
		}
	
		
		@Override
		public void onServiceConnected(ComponentName name, IBinder service) {
			Log.v(TAG, "Binder service connected");
			
			mBinder =  (LocalBinder) service;
			
			adk = mBinder.getADK();
			
			adk.setOnPublicationCB(new TopicRegistrationListener() {
				
				@Override
				public void onNewTopic(TopicInfo arg0) {
					TextView v = (TextView) findViewById(R.id.PublicationsView);
					TopicInfo[] topics = adk.getPublications();
					displayTopics(v, "Publications", topics);
				}
			});
			
			adk.setOnSubscriptionCB(new TopicRegistrationListener() {
				
				@Override
				public void onNewTopic(TopicInfo arg0) {
					TextView v = (TextView) findViewById(R.id.SubscriptionsView);
					TopicInfo[] topics = adk.getSubscriptions();
					displayTopics(v, "Subscriptions", topics);
				}
			});
						
			
			if (adk.isConnected()) {
				setADKConnected();
				
				displayTopics( (TextView) findViewById(R.id.PublicationsView),
				     "Publications", adk.getPublications()) ;
				
				displayTopics( (TextView) findViewById(R.id.SubscriptionsView),
					     "Subscriptions", adk.getSubscriptions()) ;
			}
			else{
				adk.setOnConnectonListener(new ROSSerialADK.onConnectionListener() {
					@Override
					public void trigger(boolean connection) {
						TextView v = (TextView) findViewById(R.id.ConncectionStatusView);
						if (connection) v.post(new TextViewHandler(v,"ADK Is Connected"));
						else v.post(new TextViewHandler(v, "ADK is Disconnected"));
					}
				});
			}
			

		}
	};
}                     
