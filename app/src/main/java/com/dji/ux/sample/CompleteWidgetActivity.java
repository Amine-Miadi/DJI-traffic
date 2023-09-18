package com.dji.ux.sample;


import static android.view.ViewGroup.LayoutParams.WRAP_CONTENT;
import static dji.common.flightcontroller.flightassistant.SmartTrackSubMode.SMART_TRACK_ZOOM_FREE;
import static dji.common.flightcontroller.flightassistant.SmartTrackSubMode.find;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.UUID;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.graphics.Point;
import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.InputType;
import android.text.TextUtils;
import android.view.Display;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.RelativeLayout;
import android.widget.TextView;
import android.widget.Toast;


import androidx.annotation.NonNull;

import dji.common.camera.LaserMeasureInformation;
import dji.common.camera.SettingsDefinitions;
import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.LocationCoordinate3D;
import dji.common.flightcontroller.flightassistant.SmartTrackInfo;
import dji.common.flightcontroller.flightassistant.SmartTrackState;
import dji.common.flightcontroller.flightassistant.SmartTrackTrackInfo;
import dji.common.gimbal.GimbalState;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.util.*;
import dji.sdk.camera.Camera;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.media.DownloadListener;
import dji.sdk.media.MediaFile;
import dji.sdk.mission.MissionControl;



import dji.sdk.media.MediaManager;

import dji.sdk.mission.smarttrack.SmartTrackOperator;
import dji.sdk.mission.smarttrack.SmartTrackOperatorListener;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;
import dji.ux.panel.CameraSettingAdvancedPanel;
import dji.ux.panel.CameraSettingExposurePanel;
import dji.ux.widget.FPVOverlayWidget;
import dji.ux.widget.FPVWidget;
import dji.ux.widget.ThermalPaletteWidget;
import dji.ux.widget.config.CameraConfigApertureWidget;
import dji.ux.widget.config.CameraConfigEVWidget;
import dji.ux.widget.config.CameraConfigISOAndEIWidget;
import dji.ux.widget.config.CameraConfigSSDWidget;
import dji.ux.widget.config.CameraConfigShutterWidget;
import dji.ux.widget.config.CameraConfigStorageWidget;
import dji.ux.widget.config.CameraConfigWBWidget;
import dji.ux.widget.controls.CameraControlsWidget;
import dji.ux.widget.controls.LensControlWidget;
import okhttp3.MediaType;
import okhttp3.MultipartBody;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.RequestBody;
import okhttp3.Response;

/**
 * Activity that shows all the UI elements together
 */
public class CompleteWidgetActivity extends Activity {
    private ViewGroup parentView;
    private FPVWidget fpvWidget;
    private FPVOverlayWidget fpvOverlayWidget;
    private RelativeLayout primaryVideoView;
    private CameraSettingExposurePanel cameraSettingExposurePanel;
    private CameraSettingAdvancedPanel cameraSettingAdvancedPanel;
    private CameraConfigISOAndEIWidget cameraConfigISOAndEIWidget;
    private CameraConfigShutterWidget cameraConfigShutterWidget;
    private CameraConfigApertureWidget cameraConfigApertureWidget;
    private CameraConfigEVWidget cameraConfigEVWidget;
    private CameraConfigWBWidget cameraConfigWBWidget;
    private CameraConfigStorageWidget cameraConfigStorageWidget;
    private CameraConfigSSDWidget cameraConfigSSDWidget;
    private CameraControlsWidget controlsWidget;
    private LensControlWidget lensControlWidget;
    private ThermalPaletteWidget thermalPaletteWidget;
    private static Handler handler;



    private int height;
    private int width;
    private int margin;
    private int deviceWidth;
    private int deviceHeight;
    private int speed_limit = 100;
    private double recorded_speed = 0;
    private int lookAtTarget;
    private SmartTrackOperator ST_operator;
    private Gimbal gimbal;
    static double laserDistance;
    static Aircraft aircraft;
    static LocationCoordinate3D aircraftcords;
    static List<SmartTrackTrackInfo> targetList;

    static List<MediaFile> mediaList;
    private Camera camera;
    //cartesian coordinates of the target
    private double[] targetxyz = new double[3];
    private double pitch,yaw;

    private double startPitch,startYaw;
    //target screen coordinates
    private double targetX = -1,targetY = -1;
    private MediaManager mediaManager;
    private File file;
    //tuning params
    private int measurementtime = 200, n_measurements = 7, measurementerr = 5, speedLimit = 100, minD = 400,maxD = 700;
    private double minangle,maxangle;
    private boolean tracktype; //0 is manual, 1 is auto
    private boolean selecting; // whether target is still being selected
    DecimalFormat df = new DecimalFormat("#.##");


    //thread for running the get speed method
    private Runnable startSpeedMonitoring = new Runnable() {
        @Override
        public void run() {

            double[] coord1,coord2;
            double distance,t1,t2;          //distance from drone to aircraft and the times it was measured at
            double lat1=0,lat2=0,lng1=0,lng2=0,h1=0,h2=0;   //aircraft position at first and second measure of distance

            while(true) {
                double[] speeds;
                FileOutputStream os;
                if(targetList.size()>0 && tracktype && !selecting && ST_operator.getCurrentState() == SmartTrackState.WAIT_TO_SELECT){
                    ST_operator.selectManualTarget(targetList.get(0).getBoundInfo(),selectTargetCallback);
                    selecting = true;
                }
                if(ST_operator.getCurrentState() != SmartTrackState.SPOTLIGHT)continue;
                //giving ST time to focus on target
                try {
                    speedsThread.sleep(1500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                try {
                    try {
                        os = new FileOutputStream(file, true);
                        os.write(("Starting Measurements: \n\n").getBytes());
                        os.close();
                    } catch (Exception e) {
                    }
                    speeds = new double[n_measurements];
                    for (int j = 0; j < n_measurements;j++) {
                        //this should be if target is at the center of the screen and should be timed
                        t1 = 0;t2 = 0;
                        do{
                            coord1 = new double[3];
                            coord1 = targetxyz.clone();
                            lat1 = aircraftcords.getLatitude();
                            lng1 = aircraftcords.getLatitude();
                            h1 = aircraftcords.getAltitude();
                            t1 = System.currentTimeMillis();
                        }while(coord1[0] == 0 && coord1[1] == 0 && coord1[2] == 0);

                        try {
                            os = new FileOutputStream(file, true);
                            os.write(("Start measure 1 :\nlaserDistance1: "
                                    + laserDistance + "\npitch1: " + pitch + "\nyaw1: " + yaw
                                    + "\nΔx: " + (targetList.get(0).getBoundInfo().getCenterX()-0.5) + "\nΔy: " + (targetList.get(0).getBoundInfo().getCenterY()-0.5)
                                    + "\nEnd measure 1\n\n").getBytes());
                            os.close();
                        } catch (Exception e) {
                            os = new FileOutputStream(file, true);
                            os.write(e.toString().getBytes());
                            os.close();
                        }

                        speedsThread.sleep(measurementtime);

                        try {
                            os = new FileOutputStream(file, true);
                            os.write(("---- time: " + measurementtime + "----").getBytes());
                            os.close();
                        } catch (Exception e) {
                            os = new FileOutputStream(file, true);
                            os.write(e.toString().getBytes());
                            os.close();
                        }

                        do{
                            coord2 = new double[3];
                            coord2 = targetxyz.clone();
                            lat2 = aircraftcords.getLatitude();
                            lng2 = aircraftcords.getLatitude();
                            h2 = aircraftcords.getAltitude();
                            t2 = System.currentTimeMillis();
                        }while(coord1[0] == 0 && coord1[1] == 0 && coord1[2] == 0);

                        try {
                            os = new FileOutputStream(file, true);
                            os.write(("\n\nStart Measure2: \nlaserDistance2: "
                                    + laserDistance + "\npitch2: " + pitch + "\nyaw2: " + yaw
                                    +"\nΔx: " + (targetList.get(0).getBoundInfo().getCenterX()-0.5) + "\nΔy: " + (targetList.get(0).getBoundInfo().getCenterY()-0.5)
                                    + "\nEnd Measure 2").getBytes());
                            os.close();
                        } catch (Exception e) {
                            os = new FileOutputStream(file, true);
                            os.write(e.toString().getBytes());
                            os.close();
                        }


                        distance = Math.sqrt(Math.pow(coord2[0] - coord1[0], 2) + Math.pow(coord2[1] - coord1[1], 2) + Math.pow(coord2[2] - coord1[2], 2));
                        speeds[j] = (distance / (t2-t1)) * 3600;
                        double mouvement_error = harvestineFormula(lat1,lng1,lat2,lng2,h1,h2);
                        if(speeds[j] > 350 || harvestineFormula(lat1,lng1,lat2,lng2,h1,h2) > 2){ // threasholds for controlling values
                            j--;continue;
                        }
                        try {
                            os = new FileOutputStream(file, true);
                            os.write(("\n |Haverstine| " +  mouvement_error).getBytes());
                            os.write(("\n |time| " +  (t2-t1)).getBytes());
                            if(ST_operator.getCurrentState() == SmartTrackState.SPOTLIGHT) {
                                os.write(("\n |velocity| " + df.format(Math.sqrt(Math.pow(targetList.get(0).getVelocityInfo().getEast(), 2) + Math.pow(targetList.get(0).getVelocityInfo().getNorth(), 2) + Math.pow(targetList.get(0).getVelocityInfo().getUp(), 2))*3.6)).getBytes());
                            }
                            os.write(("\n |speed| " +  speeds[j] + "\n"  + "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n").getBytes());

                            os.close();
                        } catch (Exception e) {
                            os = new FileOutputStream(file, true);
                            os.write(e.toString().getBytes());
                            os.close();
                        }
                    }

                    Arrays.sort(speeds);
                    recorded_speed = speeds[n_measurements /2];
                    try {
                        os = new FileOutputStream(file, true);
                        os.write(("\n || " +  Calendar.getInstance().getTime() + " || \n").getBytes());
                        os.write(("End Of Measurements: \n\n" + "\n || " +  Calendar.getInstance().getTime() + " || \n").getBytes());
                        os.write(("\n# # # #  speeds: " + Arrays.toString(speeds) + "# # # #   :: " +speeds[n_measurements/2]+ "\n\n\n\n").getBytes());
                        os.close();
                    } catch (Exception e) {
                        os = new FileOutputStream(file, true);
                        os.write(e.toString().getBytes());
                        os.close();
                    }

                    //reporting a violation
                    if ( recorded_speed - measurementerr > speed_limit && ST_operator.getCurrentState() == SmartTrackState.SPOTLIGHT) {
                        camera.startShootPhoto(imagecallback);
                        ToneGenerator toneGen1 = new ToneGenerator(AudioManager.STREAM_MUSIC, 100);
                        toneGen1.startTone(ToneGenerator.TONE_CDMA_PIP,600);
                        CompleteWidgetActivity.this.runOnUiThread(()->{
                            Toast.makeText(CompleteWidgetActivity.this, "Captured a vehicle going: " + recorded_speed + " km/h in a " + df.format(speed_limit) + " zone", Toast.LENGTH_SHORT).show();
                        });
                    }
                    //displaying speed
                    secondDisplay(df.format(recorded_speed) + " km/h");
                    if(tracktype){
                        ST_operator.exitSmartTrack(exitSmartTrackCallback);
                        rotateToAngle((Math.toDegrees(minangle)-90),0,RotationMode.ABSOLUTE_ANGLE);
                    }
                    speedsThread.sleep(300);

                }catch (Exception e) {
                }
            }
        }
    };
    private Thread speedsThread = new Thread(startSpeedMonitoring);


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_default_widgets);
        //init the buttons
        Button Button1 = findViewById(R.id.button1);
        Button1.setOnClickListener(view -> lookATarget1(view));

        Button Button2 = findViewById(R.id.button2);
        Button2.setOnClickListener(view -> lookATarget2(view));

        Button Button3 = findViewById(R.id.button3);
        Button3.setOnClickListener(view -> lookATarget3(view));

        Button Button4 = findViewById(R.id.button4);
        Button4.setOnClickListener(view -> lookATarget4(view));

        Button Button5 = findViewById(R.id.button5);
        Button5.setOnClickListener(view -> setSpeedErr(view));

        Button Button6 = findViewById(R.id.button6);
        Button6.setOnClickListener(view -> setTime(view));

        Button Button7 = findViewById(R.id.button7);
        Button7.setOnClickListener(view -> setMeasurementsNumber(view));

        Button Button8 = findViewById(R.id.button8);
        Button8.setOnClickListener(view -> setMinDist(view));

        Button Button9 = findViewById(R.id.button9);
        Button9.setOnClickListener(view -> setMaxDist(view));

        Button Button10 = findViewById(R.id.button10);
        Button10.setOnClickListener(view -> setSpeedLimit(view));

        Button Button11 = findViewById(R.id.button11);
        Button11.setOnClickListener(view -> autoTrack(view));

        Button Button12 = findViewById(R.id.button12);
        Button12.setOnClickListener(view -> manualTrack(view));

        TextView minDist = findViewById(R.id.minDist);
        minDist.setText(minD + "m");

        TextView maxDist = findViewById(R.id.maxDist);
        maxDist.setText(maxD + "m");

        TextView measurements_number = findViewById(R.id.deviation);
        measurements_number.setText(String.valueOf(n_measurements));

        TextView measurement_time = findViewById(R.id.measurementtime);
        measurement_time.setText(measurementtime + "ms");

        TextView speedlimit = findViewById(R.id.speed_limit);
        speedlimit.setText(speedLimit + "km/h");

        TextView m_err = findViewById(R.id.measurementerr);
        m_err.setText(measurementerr + "km/h");


        try {
            File dir = new File (getApplicationContext().getExternalFilesDir(null).toString());
            file = new File(dir, "logs" + Calendar.getInstance().getTime() +".txt");
            file.createNewFile();
        } catch (IOException e) {
            Toast.makeText(this, e.toString(), Toast.LENGTH_SHORT).show();
        }


        height = DensityUtil.dip2px(this, 100);
        width = DensityUtil.dip2px(this, 150);
        margin = DensityUtil.dip2px(this, 12);


        // getting the device dimentions
        WindowManager windowManager = (WindowManager) getSystemService(Context.WINDOW_SERVICE);
        final Display display = windowManager.getDefaultDisplay();
        Point outPoint = new Point();
        display.getRealSize(outPoint);
        deviceHeight = outPoint.y;
        deviceWidth = outPoint.x;


        initCameraView();
        parentView = (ViewGroup) findViewById(R.id.root_view);

        fpvWidget = findViewById(R.id.fpv_widget);
        fpvOverlayWidget = findViewById(R.id.fpv_overlay_widget);
        primaryVideoView = findViewById(R.id.fpv_container);

        //set the the listener to primarycamera and init secondary camera
        fpvWidget.setCameraIndexListener((cameraIndex, lensIndex) -> cameraWidgetKeyIndexUpdated(fpvWidget.getCameraKeyIndex(), fpvWidget.getLensKeyIndex()));



        // handling date loop thread
        handler = new Handler();


        //Active/smart track init********************************
        ST_operator = MissionControl.getInstance().getSmartTrackOperator();
        ST_operator.addListener(ST_listener);
        //*******************************************************



        final Runnable runnable = new Runnable() {
            public void run() {
                try{
                    camera = DJISDKManager.getInstance().getProduct().getCamera();
                    camera.getLenses().get(0).setLaserMeasureInformationCallback(lasermeasurementcallback);
                    gimbal = DJISDKManager.getInstance().getProduct().getGimbal();
                    aircraft = (Aircraft) DJISDKManager.getInstance().getProduct();
                    mediaManager = camera.getMediaManager();
                    if(mediaManager!=null)mediaManager.addUpdateFileListStateListener(filelistStateUpdate);
                    aircraft.getFlightController().setStateCallback(flightstatecallback);
                    gimbal.setStateCallback(gimbalCallback);
                    ST_operator.enterSmartTrack(completionCallback);
                }catch(Exception e ){
                }
                if(camera==null || gimbal == null || aircraft==null)
                    handler.postDelayed(this,500);
            }
        };

        //displaying the date and time
        TextView datetime = findViewById(R.id.datetime);
        Runnable timerunnable = new Runnable() {
            @Override
            public void run() {
                try{
                    Date currentTime = Calendar.getInstance().getTime();
                    datetime.setText(currentTime.toString());
                    handler.postDelayed(this,1000);
                }catch(Exception e){
                    CompleteWidgetActivity.this.runOnUiThread(()-> Toast.makeText(CompleteWidgetActivity.this, e.toString(), Toast.LENGTH_SHORT).show());
                }
            }
        };
        //init parameters handler and the calendar update handler
        handler.post(runnable);
        handler.post(timerunnable);

    }


    private void initCameraView() {
        cameraSettingExposurePanel = findViewById(R.id.camera_setting_exposure_panel);
        cameraSettingAdvancedPanel = findViewById(R.id.camera_setting_advanced_panel);
        cameraConfigISOAndEIWidget = findViewById(R.id.camera_config_iso_and_ei_widget);
        cameraConfigShutterWidget = findViewById(R.id.camera_config_shutter_widget);
        cameraConfigApertureWidget = findViewById(R.id.camera_config_aperture_widget);
        cameraConfigEVWidget = findViewById(R.id.camera_config_ev_widget);
        cameraConfigWBWidget = findViewById(R.id.camera_config_wb_widget);
        cameraConfigStorageWidget = findViewById(R.id.camera_config_storage_widget);
        cameraConfigSSDWidget = findViewById(R.id.camera_config_ssd_widget);
        lensControlWidget = findViewById(R.id.camera_lens_control);
        controlsWidget = findViewById(R.id.CameraCapturePanel);
        thermalPaletteWidget = findViewById(R.id.thermal_pallette_widget);
    }



    //binding fpv camera to the UI widgets(iso, wb....)
    private void cameraWidgetKeyIndexUpdated(int keyIndex, int subKeyIndex) {
        controlsWidget.updateKeyOnIndex(keyIndex, subKeyIndex);
        cameraSettingExposurePanel.updateKeyOnIndex(keyIndex, subKeyIndex);
        cameraSettingAdvancedPanel.updateKeyOnIndex(keyIndex, subKeyIndex);
        cameraConfigISOAndEIWidget.updateKeyOnIndex(keyIndex, subKeyIndex);
        cameraConfigShutterWidget.updateKeyOnIndex(keyIndex, subKeyIndex);
        cameraConfigApertureWidget.updateKeyOnIndex(keyIndex, subKeyIndex);
        cameraConfigEVWidget.updateKeyOnIndex(keyIndex, subKeyIndex);
        cameraConfigWBWidget.updateKeyOnIndex(keyIndex, subKeyIndex);
        cameraConfigStorageWidget.updateKeyOnIndex(keyIndex, subKeyIndex);
        cameraConfigSSDWidget.updateKeyOnIndex(keyIndex, subKeyIndex);
        controlsWidget.updateKeyOnIndex(keyIndex, subKeyIndex);
        lensControlWidget.updateKeyOnIndex(keyIndex, subKeyIndex);
        thermalPaletteWidget.updateKeyOnIndex(keyIndex, subKeyIndex);
        fpvOverlayWidget.updateKeyOnIndex(keyIndex, subKeyIndex);
    }



    @Override
    protected void onResume() {
        super.onResume();

        // Hide both the navigation bar and the status bar.
        View decorView = getWindow().getDecorView();
        decorView.setSystemUiVisibility(View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_FULLSCREEN
                | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);

    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    @Override
    protected void onSaveInstanceState(Bundle outState) {
        super.onSaveInstanceState(outState);
    }

    @Override
    public void onLowMemory() {
        super.onLowMemory();
    }



    public void secondDisplay(String i){
        TextView distance = findViewById(R.id.distance);
        CompleteWidgetActivity.this.runOnUiThread(() -> distance.setText(i));
    }
    public void mainDisplay(String i){
        TextView date  = findViewById(R.id.dateText);
        CompleteWidgetActivity.this.runOnUiThread(() -> date.setText(i));
    }
    CommonCallbacks.CompletionCallback completionCallback = djiError -> {};
    CommonCallbacks.CompletionCallback targetConfirmCompletionCallback = new CommonCallbacks.CompletionCallback() {
        @Override
        public void onResult(DJIError djiError) {
            selecting = false;
        }
    };
    LaserMeasureInformation.Callback lasermeasurementcallback = new LaserMeasureInformation.Callback() {
        @Override
        public void onUpdate(LaserMeasureInformation laserMeasureInformation) {
            if (laserMeasureInformation != null && ST_operator.getCurrentState() == SmartTrackState.SPOTLIGHT) {
                if(targetList.get(0).getBoundInfo().getCenterX() - 0.5 > 0.03 || targetList.get(0).getBoundInfo().getCenterY() - 0.5 > 0.03)
                    laserDistance = 0;
                else
                    laserDistance = laserMeasureInformation.getTargetDistance();
            }
        }
    };
    SmartTrackOperatorListener ST_listener = new SmartTrackOperatorListener() {
        String targets = "";
        List<Integer> IDs = new ArrayList<>();
        @Override
        public void onTargetInfoUpdate(List<SmartTrackTrackInfo> list) {
            CompleteWidgetActivity.this.runOnUiThread(() -> {
                try{
                    if(list != null){
                        targetList = list;
                        FrameLayout layout = findViewById(R.id.targets_tags);
                        if(list.size() > 0){
                            targets += "Operator_State: " + ST_operator.getCurrentState()
                                    + "\nOperator_Mode: " + ST_operator.getCurrentNavigationMode() + "\n";
                            layout.bringToFront();
                            //extracting IDs
                            IDs.clear();
                            for(SmartTrackTrackInfo t : list){IDs.add(t.getId());}
                            //removing non existant IDs
                            for(int i=0;i<layout.getChildCount();i++){
                                final View view = layout.getChildAt(i);
                                if(!IDs.contains(view.getId())){
                                    layout.removeView(view);
                                }
                            }
                            for(SmartTrackTrackInfo t : list){
                                if(t == null)continue;
                                //getting target info
                                targetX = t.getBoundInfo().getCenterX();
                                targetY = t.getBoundInfo().getCenterY();
                                targets += t.getType().toString() + "   |   " +t.getId()
                                        +"\n###################################\n";
                                if(ST_operator.getCurrentState() == SmartTrackState.SPOTLIGHT)
                                    targets += "\n speed: " + df.format(Math.sqrt(Math.pow(t.getVelocityInfo().getEast(),2)+Math.pow(t.getVelocityInfo().getNorth(),2)+Math.pow(t.getVelocityInfo().getUp(),2))*3.6);

                                if(findViewById(t.getId()) != null) {
                                    TextView targetTextView = findViewById(t.getId());
                                    FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT);
                                    params.leftMargin = (int) (targetX * deviceWidth) - 100;
                                    params.topMargin = (int) (targetY * deviceHeight) - 50;
                                    targetTextView.setLayoutParams(params);
                                }
                                else {
                                    TextView targetTextView = new TextView(CompleteWidgetActivity.this);
                                    targetTextView.setText(String.valueOf(t.getId()));
                                    targetTextView.setId(t.getId());
                                    targetTextView.setBackgroundColor(Color.GRAY);
                                    targetTextView.setHeight(70);
                                    targetTextView.setWidth(175);
                                    targetTextView.setVisibility(View.VISIBLE);
                                    targetTextView.setGravity(Gravity.CENTER);
                                    targetTextView.setOnClickListener(view -> track(t.getId()));
                                    FrameLayout.LayoutParams params = new FrameLayout.LayoutParams(WRAP_CONTENT, WRAP_CONTENT);
                                    params.leftMargin = (int) (targetX * deviceWidth) - 100;
                                    params.topMargin = (int) (targetY * deviceHeight) - 50;
                                    targetTextView.setLayoutParams(params);
                                    layout.addView(targetTextView);
                                    layout.bringChildToFront(targetTextView);
                                }
                            }
                            mainDisplay(targets );
                            targets = "";
                        }else{
                            targetX = -1;targetY = -1;
                            layout.removeAllViews();
                            mainDisplay("no targets detected.");
                        }
                    }
                }catch(Exception e){
                }
            });
        }

        @Override
        public void onInfoUpdate(SmartTrackInfo smartTrackInfo) {

        }
    };
    GimbalState.Callback gimbalCallback = new GimbalState.Callback() {
        @Override
        public void onUpdate(@NonNull GimbalState gimbalState) {
            //getting yaw and pitch in degrees
            pitch = gimbalState.getAttitudeInDegrees().getPitch();
            yaw = gimbalState.getYawRelativeToAircraftHeading();
            double distance = laserDistance;
            double p = Math.toRadians(90 - pitch);
            double y = -Math.toRadians(yaw);
            //getting the cartesian coordinates of the object in focus(at the center of the screen)
            targetxyz[0] = Math.sin(y) * Math.sin(p)*distance;
            targetxyz[1] = Math.cos(y) * Math.sin(p)*distance;
            targetxyz[2] = Math.cos(p)*distance;

        }
    };
    FlightControllerState.Callback flightstatecallback = new FlightControllerState.Callback() {
         @Override
         public void onUpdate(@NonNull FlightControllerState flightControllerState) {
            aircraftcords = flightControllerState.getAircraftLocation();
         }
     };
    double harvestineFormula(double lat1, double lon1, double lat2, double lon2, double h1, double h2){
         //height
         double h = Math.abs(h2-h1);
         // distance between latitudes and longitudes
         double dLat = Math.toRadians(lat2 - lat1);
         double dLon = Math.toRadians(lon2 - lon1);

         // convert to radians
         lat1 = Math.toRadians(lat1);
         lat2 = Math.toRadians(lat2);

         // apply formulae
         double a = Math.pow(Math.sin(dLat / 2), 2) + Math.pow(Math.sin(dLon / 2), 2) * Math.cos(lat1) * Math.cos(lat2);
         double rad = 6371; //radius of the earth (mean)
         double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
         double planeDist = rad * c;
         return Math.sqrt(Math.pow(planeDist,2) + Math.pow(h,2));
     };
     //button callbacks
    public void lookATarget1(View view){
        if(targetList.size() > 0) {
            double x = targetList.get(0).getBoundInfo().getCenterX();
            double y = targetList.get(0).getBoundInfo().getCenterY();
            rotateToAngle(-(y-0.5) * 40, (x-0.5) * 51,RotationMode.RELATIVE_ANGLE);
        }
    }
    public void lookATarget2(View view){
        camera.getLenses().get(0).setHybridZoomFocalLength(317,completionCallback);
    }
    public void lookATarget3(View view){
        if(targetList.size()>2)
            ST_operator.selectManualTarget(targetList.get(1).getBoundInfo(),selectTargetCallback);
    }
    public void lookATarget4(View view){
        ST_operator.exitSmartTrack(exitSmartTrackCallback);
    }
    public void setSpeedErr(View view){
        try {
            CompleteWidgetActivity.this.runOnUiThread(() ->{
                TextView field = findViewById(R.id.measurementerr);
                AlertDialog.Builder builder = new AlertDialog.Builder(CompleteWidgetActivity.this);
                builder.setTitle("Input the allowed speed measurement error (km/h)");
                final EditText input = new EditText(CompleteWidgetActivity.this);
                input.setInputType(InputType.TYPE_CLASS_NUMBER);
                builder.setView(input);
                builder.setPositiveButton("OK", null);
                AlertDialog dialog = builder.show();
                Button button = dialog.getButton(AlertDialog.BUTTON_POSITIVE);
                button.setOnClickListener(view1 -> {
                    String m_Text = input.getText().toString().trim();
                    if(TextUtils.isEmpty(m_Text)){
                        input.setError("Please Enter a value");
                    } else {
                        measurementerr = Integer.valueOf(m_Text);
                        field.setText(measurementerr + " km/h ");
                        dialog.dismiss();
                    }
                });
            });
        }catch(Exception e ){
            Toast.makeText(CompleteWidgetActivity.this, e.toString(), Toast.LENGTH_SHORT).show();
        }
    }
    public void setMinDist(View view){
        try {
            CompleteWidgetActivity.this.runOnUiThread(() ->{
                TextView field = findViewById(R.id.minDist);
                AlertDialog.Builder builder = new AlertDialog.Builder(CompleteWidgetActivity.this);
                builder.setTitle("Input the minimum distance to measure (meters)");
                final EditText input = new EditText(CompleteWidgetActivity.this);
                input.setInputType(InputType.TYPE_CLASS_NUMBER);
                builder.setView(input);
                builder.setPositiveButton("OK", null);
                AlertDialog dialog = builder.show();
                Button button = dialog.getButton(AlertDialog.BUTTON_POSITIVE);
                button.setOnClickListener(view1 -> {
                    String m_Text = input.getText().toString().trim();
                    if(TextUtils.isEmpty(m_Text)){
                        input.setError("Please Enter a value");
                    } else {
                        minD = Integer.valueOf(m_Text);
                        minangle = Math.atan(minD/aircraftcords.getAltitude());
                        field.setText(df.format(Math.toDegrees(minangle))  + "° | " +minD + " m ");
                        dialog.dismiss();
                    }
                });
            });
        }catch(Exception e ){
            Toast.makeText(CompleteWidgetActivity.this, e.toString(), Toast.LENGTH_SHORT).show();
        }
    }
    public void setMaxDist(View view){
        try {
            CompleteWidgetActivity.this.runOnUiThread(() ->{
                TextView field = findViewById(R.id.maxDist);
                AlertDialog.Builder builder = new AlertDialog.Builder(CompleteWidgetActivity.this);
                builder.setTitle("Input the maximum distance to measure (meters)");
                final EditText input = new EditText(CompleteWidgetActivity.this);
                input.setInputType(InputType.TYPE_CLASS_NUMBER);
                builder.setView(input);
                builder.setPositiveButton("OK", null);
                AlertDialog dialog = builder.show();
                Button button = dialog.getButton(AlertDialog.BUTTON_POSITIVE);
                button.setOnClickListener(view1 -> {
                    String m_Text = input.getText().toString().trim();
                    if(TextUtils.isEmpty(m_Text)){
                        input.setError("Please Enter a value");
                    } else {
                        maxD = Integer.valueOf(m_Text);
                        maxangle = Math.atan(maxD/aircraftcords.getAltitude());
                        field.setText(df.format(Math.toDegrees(maxangle))  + "° | " +maxD + " m ");
                        dialog.dismiss();
                    }
                });
            });
        }catch(Exception e ){
            Toast.makeText(CompleteWidgetActivity.this, e.toString(), Toast.LENGTH_SHORT).show();
        }
    }
    public void setTime(View view){
        try {
            CompleteWidgetActivity.this.runOnUiThread(() ->{
                TextView field = findViewById(R.id.measurementtime);
                AlertDialog.Builder builder = new AlertDialog.Builder(CompleteWidgetActivity.this);
                builder.setTitle("Input the time interval between measurements (ms)");
                final EditText input = new EditText(CompleteWidgetActivity.this);
                input.setInputType(InputType.TYPE_CLASS_NUMBER);
                builder.setView(input);
                builder.setPositiveButton("OK", null);
                AlertDialog dialog = builder.show();
                Button button = dialog.getButton(AlertDialog.BUTTON_POSITIVE);
                button.setOnClickListener(view1 -> {
                    String m_Text = input.getText().toString().trim();
                    if(TextUtils.isEmpty(m_Text)){
                        input.setError("Please Enter a value");
                    }else if(Integer.valueOf(m_Text) < 100){
                        input.setError("The value must be higher than 100ms");
                    } else {
                        measurementtime = Integer.valueOf(m_Text);
                        field.setText(measurementtime + " ms ");
                        dialog.dismiss();
                    }
                });
            });
        }catch(Exception e ){
            Toast.makeText(CompleteWidgetActivity.this, e.toString(), Toast.LENGTH_SHORT).show();
        }
    }
    public void setMeasurementsNumber(View view){
        try {
            CompleteWidgetActivity.this.runOnUiThread(() ->{
                TextView field = findViewById(R.id.deviation);
                AlertDialog.Builder builder = new AlertDialog.Builder(CompleteWidgetActivity.this);
                builder.setTitle("Input the number of measurements for one speed estimation");
                final EditText input = new EditText(CompleteWidgetActivity.this);
                input.setInputType(InputType.TYPE_CLASS_NUMBER);
                builder.setView(input);
                builder.setPositiveButton("OK", null);
                AlertDialog dialog = builder.show();
                Button button = dialog.getButton(AlertDialog.BUTTON_POSITIVE);
                button.setOnClickListener(view1 -> {
                    String m_Text = input.getText().toString().trim();
                    if(TextUtils.isEmpty(m_Text)){
                        input.setError("Please Enter a value");
                    } else {
                        n_measurements = Integer.valueOf(m_Text);
                        field.setText(n_measurements + "   ");
                        dialog.dismiss();
                    }
                });
            });
        }catch(Exception e ){
            Toast.makeText(CompleteWidgetActivity.this, e.toString(), Toast.LENGTH_SHORT).show();
        }
    }
    public void setSpeedLimit(View view){
        try {
            CompleteWidgetActivity.this.runOnUiThread(() ->{
                //getting the speed input
                TextView field = findViewById(R.id.speed_limit);
                AlertDialog.Builder builder = new AlertDialog.Builder(CompleteWidgetActivity.this);
                builder.setTitle("Input the speed limit");
                final EditText input = new EditText(CompleteWidgetActivity.this);
                input.setInputType(InputType.TYPE_CLASS_NUMBER);
                builder.setView(input);
                builder.setPositiveButton("OK", null);
                AlertDialog dialog = builder.show();
                Button button = dialog.getButton(AlertDialog.BUTTON_POSITIVE);
                button.setOnClickListener(view1 -> {
                    String m_Text = input.getText().toString().trim();
                    if(TextUtils.isEmpty(m_Text)){
                        input.setError("Please Enter a value");
                    } else {
                        speed_limit = Integer.valueOf(m_Text);
                        field.setText(speed_limit + " km/h ");
                        dialog.dismiss();
                    }
                });
            });
        }catch(Exception e ){
            Toast.makeText(CompleteWidgetActivity.this, e.toString(), Toast.LENGTH_SHORT).show();
        }
    }
    public void manualTrack(View view){
        tracktype = false;
        findViewById(R.id.button12).setBackgroundTintList(ColorStateList.valueOf(Color.parseColor("#4cc26b")));
        findViewById(R.id.button11).setBackgroundTintList(ColorStateList.valueOf(Color.parseColor("#ffffff")));
        camera.getLenses().get(0).setHybridZoomFocalLength(317,completionCallback);
        if(!speedsThread.isAlive())
            speedsThread.start();
    }
    public void autoTrack(View view){
        tracktype = true;
        minangle = Math.atan(minD/aircraftcords.getAltitude());
        maxangle = Math.atan(maxD/aircraftcords.getAltitude());
        findViewById(R.id.button11).setBackgroundTintList(ColorStateList.valueOf(Color.parseColor("#4cc26b")));
        findViewById(R.id.button12).setBackgroundTintList(ColorStateList.valueOf(Color.parseColor("#ffffff")));
        camera.getLenses().get(0).setHybridZoomFocalLength(317,completionCallback);
        if(!speedsThread.isAlive())
            speedsThread.start();
    }
    public CommonCallbacks.CompletionCallback fetchMediacallback = new CommonCallbacks.CompletionCallback() {
        @Override
        public void onResult(DJIError djiError) {
            mediaList = mediaManager.getSDCardFileListSnapshot();
            if(mediaList.size()>0)
                mediaList.get(0).fetchFileByteData(0,new downloadListener());
        }
    };
    public CommonCallbacks.CompletionCallback imagecallback = new CommonCallbacks.CompletionCallback() {
        @Override
        public void onResult(DJIError djiError) {
            handler.postDelayed(() -> mediaManager.refreshFileListOfStorageLocation(SettingsDefinitions.StorageLocation.SDCARD,fetchMediacallback),2000);
        }
    };
    public class downloadListener<E> implements DownloadListener<E> {
        List<byte[]> imageChunks = new ArrayList<>();
        @Override
        public void onStart() {
        }

        @Override
        public void onRateUpdate(long l, long l1, long l2) {

        }

        @Override
        public void onRealtimeDataUpdate(byte[] data, long l, boolean b) {
            imageChunks.add(data);
        }

        @Override
        public void onProgress(long total, long current) {
        }

        @Override
        public void onSuccess(E data) {
            int totaLength = 0,last = 0;
            for(byte[] chunk : imageChunks)totaLength += chunk.length;
            byte[] img = new byte[totaLength];
            for(byte[] chunk : imageChunks){
                for(byte b : chunk){
                    img[last] = b;
                    last++;
                }
            }
            try{
                POST_image(img);
            }catch(Exception e){
                Toast.makeText(CompleteWidgetActivity.this, e.toString(), Toast.LENGTH_SHORT).show();
            }
        }

        @Override
        public void onFailure(DJIError djiError) {
        }
    };
    public CommonCallbacks.CompletionCallback selectTargetCallback = new CommonCallbacks.CompletionCallback() {
        @Override
        public void onResult(DJIError djiError) {
            FileOutputStream os;
            try {
                os = new FileOutputStream(file, true);
                os.write(("\n\nStart Tracking\n\n").getBytes());
                os.close();
            } catch (Exception e) {
            }
            handler.postDelayed(()->ST_operator.confirmTarget(SMART_TRACK_ZOOM_FREE,targetConfirmCompletionCallback),100);
        }
    };
    public CommonCallbacks.CompletionCallback exitSmartTrackCallback = new CommonCallbacks.CompletionCallback() {
        @Override
        public void onResult(DJIError djiError) {
            FileOutputStream os;
            try {
                os = new FileOutputStream(file, true);
                os.write(("\n\nStop Tracking\n\n").getBytes());
                os.close();
            } catch (Exception e) {
            }
            camera.getLenses().get(0).setHybridZoomFocalLength(317,completionCallback);
            handler.postDelayed(() -> ST_operator.enterSmartTrack(completionCallback),250);
        }
    };
    public MediaManager.FileListStateListener filelistStateUpdate = fileListState -> {};
    public void POST_image(byte[] img){
        final OkHttpClient client = new OkHttpClient.Builder().build();
        String uniqueID = UUID.randomUUID().toString();
        RequestBody requestBody = new MultipartBody.Builder()
                .setType(MultipartBody.FORM)
                .addFormDataPart("recorded_speed", Double.toString(recorded_speed))
                .addFormDataPart("speed_limit", Integer.toString(speed_limit))
                .addFormDataPart("image", uniqueID+".jpeg",
                        RequestBody.create(MediaType.parse("image/jpeg"), img))
                .build();
        Request request = new Request.Builder()
                .url("https://concise-emblem-395909.oa.r.appspot.com/cars")
                .post(requestBody)
                .build();

        try (Response response = client.newCall(request).execute()) {
            if (!response.isSuccessful()) throw new IOException("Unexpected code " + response);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }
    public void rotateToAngle(double pitchAngle, double yawAngle,RotationMode r_mode){
        //passing an angle to rotate gimbal to (pitch only for now)
        gimbal.rotate(new Rotation.Builder()
                .mode(r_mode)
                .pitch((float) pitchAngle) // Specify the pitch angle
                .roll(0) // Specify the roll angle
                .yaw((float) yawAngle) // Specify the yaw angle
                .time(0.01) // Specify the rotation time in seconds
                .build(),completionCallback);
    }
    public void track(int id){
        for (int i = 0; i < targetList.size(); i++) {
            if(targetList.get(i).getId() == id) {
                double x = targetList.get(i).getBoundInfo().getCenterX();
                double y = targetList.get(i).getBoundInfo().getCenterY();
                ST_operator.selectManualTarget(targetList.get(i).getBoundInfo(), selectTargetCallback);
            }
        }
    }
}