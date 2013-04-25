import processing.net.*; 
import java.util.Date;

float Z_OFFSET = 1200;

Client myClient; 
Hardware hardware;
Kinect kinect;
String dataIn; 
String last = null;
CalibrationProgram calibrationProgram = null;
Program program = null;
boolean drawInit = false;

/* Steps
 convert to scala?
 Improved ball detection.
 Finish calibration.
 Turn off enable pins on stop.
 Model torque physics.
 Experiement to discover speed as a function of position and pwm for each motor.
 Devise calibration routine for above.
 Devise hook or crook repositioning movement.
 Linear motion.
 Circular motion (off constraint planes).
 */

void setup() { 
  size(1440,810);
  smooth();

  // Connect to the local machine at port 5204.
  // This example will not run if you haven't
  // previously started a server on this port.
  myClient = new Client(this, "127.0.0.1", 5204);
  hardware = new Hardware(this);
  kinect = new Kinect(this, 
    new WindowContext("Kinect Depth", 0, 0, 360, 270),
    new WindowContext("Measurement", 0, 270, 360, 270)
    );

  frame.removeNotify();
  frame.setUndecorated(false);
  frame.addNotify();
} 

void draw() {
  int motor, p;
  float power;

  if (! drawInit) {
    frame.setLocation(0, 0);
    drawInit = true;
  }
  
  Sphere ball = kinect.locateBall();

  if ( ball != null ) {
    if ( program != null )
      if ( program.step(ball) )
        program = null;
  
    renderBall(ball);
  }
  
  while (myClient.available() > 0) { 
    dataIn = myClient.readString();

    if ( last != null )
      dataIn = last += dataIn;

    if ( ! dataIn.endsWith("\n") ) {
      last = dataIn;
      continue;
    }
        
    last = null;
    
    String[] objects = dataIn.split("\n");
    for (String object: objects) {
      if ( object.length() > 0 ) {
        try {
          JSONObject parsed = JSONObject.parse(object);

          if ( parsed.hasKey("state") ) {
            motor = 3;
            power = p = parsed.getInt("state");
          } else {
            motor = parsed.getInt("motor");
            power = parsed.getFloat("power");
            p = int(255.99999*abs(power));
          }
          hardware.sendCommand( motor, (power>0?1:0), p);         
        } catch ( Exception e ) {
          hardware.error();
          println( "error: " + dataIn ); 
        }
      }
    }
  }
}

void keyPressed() {
  if ( key == 'c' ) {
     program = calibrationProgram = new CalibrationProgram(hardware);
  }
}

void renderBall(Sphere ball) {


  /*
  background(100);
  ellipseMode(RADIUS);
  stroke(100,100,100);

  
  float s = .25;
  pushMatrix();
  line(0,120,320,120);
  line(160,0,160,240);  
  
  translate(width/2, 120);
  scale(s,-s);
  
  float mRadius = 10;
  
  fill(255);
  ellipse(ball.position.x,ball.position.y,ball.r,ball.r);
  stroke(255,0,0);
//  for (ScanLine scanLine : scanLines)
//    line(scanLine.startX, scanLine.y, scanLine.endX, scanLine.y );
  stroke(100,100,100);

  if ( calibrationProgram != null ) {
    for (BaselineProgram baselineProgram : calibrationProgram.programs) {

      fill(150,150,200);
      for ( Sphere measurement : baselineProgram.measurements )
        ellipse(measurement.position.x,measurement.position.y,mRadius,mRadius);

      if ( baselineProgram.translated.size() > 0 ) {
        pushMatrix();
        float offset = baselineProgram.translated.get(0).x;
        fill(150,255,150);
        ellipse(baselineProgram.pointA.x-offset,baselineProgram.pointA.y,2*mRadius,2*mRadius);      
        ellipse(baselineProgram.pointB.x-offset,baselineProgram.pointB.y,2*mRadius,2*mRadius);      

        fill(150,150,255);
        ellipse(baselineProgram.midpoint.x-offset,baselineProgram.midpoint.y,2*mRadius,2*mRadius);
        ellipse(baselineProgram.arcCenter.x-offset,baselineProgram.arcCenter.y,2*mRadius,2*mRadius);
        for ( Pt measurement : baselineProgram.middle )
          ellipse(measurement.x-offset,measurement.y,mRadius,mRadius);

        fill(255,150,150);
        for ( Pt measurement : baselineProgram.translated )
          ellipse(measurement.x-offset,measurement.y,mRadius,mRadius);

        noFill();
        stroke(255,255,100);
        line(baselineProgram.arcCenter.x-offset,baselineProgram.arcCenter.y,baselineProgram.center.x-offset,baselineProgram.center.y);
        ellipse(baselineProgram.center.x-offset,baselineProgram.center.y,baselineProgram.r,baselineProgram.r);
        stroke(100,100,100);
        popMatrix();
      }
    }
  }

  popMatrix();

  pushMatrix();
  line(0,360,320,360);

  translate(width/2, 360);
  scale(s,-s);
  fill(255);
  ellipse(Z_OFFSET-ball.position.z,ball.position.y,ball.r,ball.r);
  
  if ( calibrationProgram != null ) {
    for (BaselineProgram baselineProgram : calibrationProgram.programs) {
      fill(150,150,200);
      for ( Sphere measurement : baselineProgram.measurements )
        ellipse(Z_OFFSET-measurement.position.z,measurement.position.y,mRadius,mRadius);
//      if ( baselineProgram.translated.size() > 0 ) {
//        float offset = baselineProgram.translated.get(0).z;
//        fill(255,150,150);
//        for ( Sphere measurement : baselineProgram.translated )
//          ellipse(-measurement.z-offset,measurement.y,mRadius,mRadius);
//      }
    }
  }

  popMatrix();

  pushMatrix();
  line(160,480,160,720);  
  translate(width/2, 600);
  scale(.4*s,-.4*s);

  fill(255);
  ellipse(ball.position.x,Z_OFFSET-ball.position.z,ball.r,ball.r);
  if ( calibrationProgram != null ) {
    for (BaselineProgram baselineProgram : calibrationProgram.programs) {
      stroke(100,100,100);
      fill(150,150,200);
      for ( Sphere measurement : baselineProgram.measurements )
        ellipse(measurement.position.x,Z_OFFSET-measurement.position.z,mRadius,mRadius);
      if ( baselineProgram.translated.size() > 0 ) {
        float offset = baselineProgram.translated.get(0).x;
        fill(255,150,150);
        for ( Pt translated : baselineProgram.translated ) {
          Pt measurement = translated.rotateXZ(baselineProgram.theta);
          ellipse(measurement.x,Z_OFFSET-measurement.z,mRadius,mRadius);
        }
      }

      if ( baselineProgram.measurements.size() > 0 ) {
        stroke(255,150,150);
        Sphere start = baselineProgram.measurements.get(0);
        line( start.position.x, Z_OFFSET-start.position.z, start.position.x + 5*baselineProgram.ax, Z_OFFSET - start.position.z - 5*baselineProgram.az );
      }

      if ( baselineProgram.baseline != null ) {
        stroke(255,200,0);
        strokeWeight(5);
        Line scaled = baselineProgram.baseline.scale(1000);
        line( scaled.start.x, Z_OFFSET - scaled.start.z, scaled.end.x, Z_OFFSET - scaled.end.z );
        scaled = baselineProgram.baseline.scale(-1000);
        line( scaled.start.x, Z_OFFSET - scaled.start.z, scaled.end.x, Z_OFFSET - scaled.end.z );
        strokeWeight(1);
      }
    }

    fill(128);
    stroke(255,200,0);
    for (Pt eyeBolt : calibrationProgram.eyeBolts) {
      ellipse(eyeBolt.x,Z_OFFSET-eyeBolt.z,2*mRadius,2*mRadius);
    }
  }
  popMatrix();
  */
}

class WindowContext {
  String title;
  int x;
  int y;
  int width;
  int height;
  private PGraphics graphics;
  private PGraphics parent;

  WindowContext(String title, int x, int y, int width, int height) {
    this(title,x,y,width,height,null);
  }

  WindowContext(String title, int x, int y, int width, int height, PGraphics parent) {
    this.title = title;
    this.x = x;
    this.y = y;
    this.width = width;
    this.height = height;
    this.parent = parent;
    graphics = createGraphics(width, height);
  }

  PGraphics beginContext() {
    graphics.beginDraw();
    graphics.clear();
    return graphics;
  }

  void endContext() {
    graphics.endDraw();
    if (parent != null) {
      parent.fill(100);
      parent.rect(x,y,width,height);
      parent.image(graphics,x,y);
      if (title != null) {
        parent.fill(0,0,0,75);
        parent.text(title, x + 11, y +21);
        parent.fill(255,255,255,75);
        parent.text(title, x + 10, y +20);
      }
    } else {
      fill(100);
      noStroke();
      rect(x,y,width,height);
      image(graphics,x,y);
      if (title != null) {
        fill(0,0,0,75);
        text(title, x + 11, y +21);
        fill(255,255,255,75);
        text(title, x + 10, y +20);
      }      
    }
  }
}


