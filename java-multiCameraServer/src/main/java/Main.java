/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.ArrayList;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.CameraServerJNI;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Scalar;


/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
       "switched cameras": [
           {
               "name": <virtual camera name>
               "key": <network table key used for selection>
               // if NT value is a string, it's treated as a name
               // if NT value is a double, it's treated as an integer index
           }
       ]
   }
 */

public final class Main {
  private static String configFile = "/boot/frc.json";

  @SuppressWarnings("MemberName")
  public static class CameraConfig {
    public String name;
    public String path;
    public JsonObject config;
    public JsonElement streamConfig;
  }

  @SuppressWarnings("MemberName")
  public static class SwitchedCameraConfig {
    public String name;
    public String key;
  };

  public static int team;
  public static boolean server;
  public static List<CameraConfig> cameraConfigs = new ArrayList<>();
  public static List<SwitchedCameraConfig> switchedCameraConfigs = new ArrayList<>();
  public static List<VideoSource> cameras = new ArrayList<>();
  public static final int cameraResX = 1280;
  public static final int cameraResY = 720;
  private static ArrayList<Rect> printTarget;
  private static final Object imgLock = new Object();
  private static VisionThread visionThread;
  private static double centerX = 0.0;

  private Main() {
  }

  /**
   * Report parse error.
   */
  public static void parseError(final String str) {
    System.err.println("config error in '" + configFile + "': " + str);
  }

  /**
   * Read single camera configuration.
   */
  public static boolean readCameraConfig(final JsonObject config) {
    final CameraConfig cam = new CameraConfig();

    // name
    final JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    final JsonElement pathElement = config.get("path");
    if (pathElement == null) {
      parseError("camera '" + cam.name + "': could not read path");
      return false;
    }
    cam.path = pathElement.getAsString();

    // stream properties
    cam.streamConfig = config.get("stream");

    cam.config = config;

    cameraConfigs.add(cam);
    return true;
  }

  /**
   * Read single switched camera configuration.
   */
  public static boolean readSwitchedCameraConfig(final JsonObject config) {
    final SwitchedCameraConfig cam = new SwitchedCameraConfig();

    // name
    final JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read switched camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    final JsonElement keyElement = config.get("key");
    if (keyElement == null) {
      parseError("switched camera '" + cam.name + "': could not read key");
      return false;
    }
    cam.key = keyElement.getAsString();

    switchedCameraConfigs.add(cam);
    return true;
  }

  /**
   * Read configuration file.
   */
  @SuppressWarnings("PMD.CyclomaticComplexity")
  public static boolean readConfig() {
    // parse file
    JsonElement top;
    try {
      top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
    } catch (final IOException ex) {
      System.err.println("could not open '" + configFile + "': " + ex);
      return false;
    }

    // top level must be an object
    if (!top.isJsonObject()) {
      parseError("must be JSON object");
      return false;
    }
    final JsonObject obj = top.getAsJsonObject();

    // team number
    final JsonElement teamElement = obj.get("team");
    if (teamElement == null) {
      parseError("could not read team number");
      return false;
    }
    team = teamElement.getAsInt();

    // ntmode (optional)
    if (obj.has("ntmode")) {
      final String str = obj.get("ntmode").getAsString();
      if ("client".equalsIgnoreCase(str)) {
        server = false;
      } else if ("server".equalsIgnoreCase(str)) {
        server = true;
      } else {
        parseError("could not understand ntmode value '" + str + "'");
      }
    }

    // cameras
    final JsonElement camerasElement = obj.get("cameras");
    if (camerasElement == null) {
      parseError("could not read cameras");
      return false;
    }
    final JsonArray cameras = camerasElement.getAsJsonArray();
    for (final JsonElement camera : cameras) {
      if (!readCameraConfig(camera.getAsJsonObject())) {
        return false;
      }
    }

    if (obj.has("switched cameras")) {
      final JsonArray switchedCameras = obj.get("switched cameras").getAsJsonArray();
      for (final JsonElement camera : switchedCameras) {
        if (!readSwitchedCameraConfig(camera.getAsJsonObject())) {
          return false;
        }
      }
    }

    return true;
  }

  /**
   * Start running the camera.
   */
  public static VideoSource startCamera(final CameraConfig config) {
    System.out.println("Starting camera '" + config.name + "' on " + config.path);
    final CameraServer inst = CameraServer.getInstance();
    final UsbCamera camera = new UsbCamera(config.name, config.path);
    final MjpegServer server = inst.startAutomaticCapture(camera);

    final Gson gson = new GsonBuilder().create();

    camera.setConfigJson(gson.toJson(config.config));
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    if (config.streamConfig != null) {
      server.setConfigJson(gson.toJson(config.streamConfig));
    }

    return camera;
  }

  /**
   * Start running the switched camera.
   */
  public static MjpegServer startSwitchedCamera(final SwitchedCameraConfig config) {
    System.out.println("Starting switched camera '" + config.name + "' on " + config.key);
    final MjpegServer server = CameraServer.getInstance().addSwitchedCamera(config.name);

    NetworkTableInstance.getDefault().getEntry(config.key).addListener(event -> {
      if (event.value.isDouble()) {
        final int i = (int) event.value.getDouble();
        if (i >= 0 && i < cameras.size()) {
          server.setSource(cameras.get(i));
        }
      } else if (event.value.isString()) {
        final String str = event.value.getString();
        for (int i = 0; i < cameraConfigs.size(); i++) {
          if (str.equals(cameraConfigs.get(i).name)) {
            server.setSource(cameras.get(i));
            break;
          }
        }
      }
    }, EntryListenerFlags.kImmediate | EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    return server;
  }

  /**
   * Example pipeline.
   */
  public static class MyPipeline implements VisionPipeline {
    public int val;

    @Override
    public void process(final Mat mat) {
      val += 1;
    }
  }

  /**
   * Main.
   */
  public static void main(final String... args) {
    if (args.length > 0) {
      configFile = args[0];
    }

    // read configuration
    if (!readConfig()) {
      return;
    }

    // start NetworkTables
    final NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    if (server) {
      System.out.println("Setting up NetworkTables server");
      ntinst.startServer();
    } else {
      System.out.println("Setting up NetworkTables client for team " + team);
      ntinst.startClientTeam(team);
    }

    // start cameras
    for (final CameraConfig config : cameraConfigs) {
      cameras.add(startCamera(config));
    }

    // start switched cameras
    for (final SwitchedCameraConfig config : switchedCameraConfigs) {
      startSwitchedCamera(config);
    }

    int streamPort = 1173;
    System.loadLibrary("opencv_java310");
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
      inst.startClientTeam(3719);
      inst.startDSClient(streamPort);
    MjpegServer inputStream = new MjpegServer("MJPEG Server", streamPort);
    UsbCamera camera = setUsbCamera(0, inputStream);
      camera.setResolution(cameraResX, cameraResY);
      camera.setFPS(20);
    CvSink imageSink = new CvSink("CV Image Grabber");
    imageSink.setSource(camera);
    CvSource imageSource = new CvSource("CV Image Source", VideoMode.PixelFormat.kMJPEG, cameraResX, cameraResY, 20);
    MjpegServer cvStream = new MjpegServer("CV Image Stream", streamPort);
    cvStream.setSource(imageSource);
    Mat inputImage = new Mat();
    Mat frame = new Mat();
    while (true) {
      long frameTime = imageSink.grabFrame(inputImage);
      if (frameTime == 0) continue;
        imageSource.putFrame(inputImage);
      
        
        
       // UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
      System.out.println("1b");
      CameraServer.getInstance().getVideo().grabFrame(frame);
      if (!pipeline.filterContoursOutput().isEmpty()) {
        synchronized (imgLock) {
          double distanceAR = 0;
          final ArrayList<Rect> unknownTarget = new ArrayList<>();
          for (int i = 0; i < pipeline.filterContoursOutput().size(); i++) {
            final Rect target = Imgproc.boundingRect(pipeline.filterContoursOutput().get(i));
            unknownTarget.add(target);
            System.out.println("Target: " + i);
            System.out.println(target.x);
            System.out.println(target.width);
            System.out.println(target.y);
            System.out.println(target.height);
            // System.out.println("2");
          }
          printTarget = unknownTarget;
            final ArrayList<Rect> filterRect = new ArrayList<>();
          for (int i = 0; i < unknownTarget.size(); i++) {
            final double TargetPix = unknownTarget.get(i).width / unknownTarget.get(i).height;
            final double FOVPix = cameraResX * cameraResY;
            final double Targetft = 667.08;
            distanceAR = (Targetft * FOVPix) / (2 * TargetPix * 0.499);

            System.out.println("AR: " + distanceAR + ": " + i);
          }
          if(printTarget.size()==1) {
            Rect rectOne = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0)); }

          Imgproc.line(inputImage, new Point(5,50), new Point(5,50), new Scalar(255,0,0));
          for (int i = 0; i < printTarget.size(); i++) {
            final var tempRect = printTarget.get(i);
              Imgproc.rectangle(inputImage,
                  new Point(tempRect.width + tempRect.x , tempRect.height + tempRect.y),
                  new Point(tempRect.x , tempRect.y), new Scalar(255, 255, 255));
              Imgproc.line(inputImage,
                  new Point(cameraResX, tempRect.y),
                  new Point(cameraResY, tempRect.y + tempRect.height), new Scalar(255,0,0));
              double centerLine = cameraResX/2 + cameraResY/2;
              double leftx = tempRect.x + tempRect.width;
              double centerLineTarget =  tempRect.x - leftx;
              double width = tempRect.x + tempRect.width;
              double distance = distanceAR / width;
              System.out.println("centerLine:" + centerLine);
              System.out.println("width:" + width);
              System.out.println("distance:" + distance);
            SmartDashboard.putNumber("centerLine", centerLine);
           }
        }
      }

    });
    
    // System.out.println("3");

    visionThread.start();
  
    // loop forever
    for (;;) {
      try {
        Thread.sleep(10000);
      } catch (final InterruptedException ex) {
        return;
      }
    }
  }
  }
 private static UsbCamera setUsbCamera(int cameraId, MjpegServer server) {
    // This gets the image from a USB camera 
    // Usually this will be on device 0, but there are other overloads
    // that can be used
    UsbCamera camera = new UsbCamera("CameraCo", cameraId);
    server.setSource(camera);
    return camera;
  }
}

