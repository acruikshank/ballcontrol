import processing.net.*; 
import java.util.Date;

Client myClient; 
Hardware hardware;
Kinect kinect;
String dataIn; 
String last = null;
CalibrationProgram calibrationProgram = null;
Program program = null;
boolean drawInit = false;
WindowContext primaryContext;
PFont bfont;
PFont font;

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
  background(100);

  // Connect to the local machine at port 5204.
  // This example will not run if you haven't
  // previously started a server on this port.
  myClient = new Client(this, "127.0.0.1", 5204);
  hardware = new Hardware(this);
  kinect = new Kinect(this, 
    new WindowContext("Kinect Depth", 0, 0, 360, 270),
    new WindowContext("Measurement", 0, 270, 360, 270),
    new WindowContext("Results", 0, 540, 360, 270)
    );
  primaryContext = new WindowContext(null,360,0,1080,810);

  frame.removeNotify();
  frame.setUndecorated(false);
  frame.addNotify();

  font = createFont("Andale Mono", 14);
  bfont = createFont("Helvetica-Bold", 14);
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
     program = calibrationProgram = new CalibrationProgram(hardware, primaryContext);
  }
}

class WindowContext {
  int LEFT_HEADER_MARGIN = 10;
  int TOP_TEXT_MARGIN = 55;
  int LEFT_TEXT_MARGIN = 20;
  int LEADING = 30;
  String title;
  int x;
  int y;
  int width;
  int height;
  int textTop = TOP_TEXT_MARGIN;
  int textLeft = LEFT_TEXT_MARGIN;
  PGraphics graphics;
  private WindowContext parent;

  WindowContext(String title, int x, int y, int width, int height) {
    this(title,x,y,width,height,null);
  }

  WindowContext(String title, int x, int y, int width, int height, WindowContext parent) {
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
    textTop = TOP_TEXT_MARGIN;
    textLeft = LEFT_TEXT_MARGIN;
    return graphics;
  }

  void writeLine(String text) {
    graphics.textFont(font,14);
    graphics.noStroke();
    graphics.fill(230);
    graphics.text( text, textLeft, textTop );
    textTop += LEADING;
  }

  void offsetText( int x, int y ) {
    textTop += y;
    textLeft += x;
  }

  void endContext() {
    graphics.endDraw();
    if (parent != null) {
      parent.graphics.textFont(bfont,14);
      parent.graphics.fill(100);
      parent.graphics.noStroke();
      parent.graphics.rect(x,y,width,height);
      parent.graphics.image(graphics,x,y);
      if (title != null) {
        parent.graphics.fill(0,0,0,75);
        parent.graphics.text(title, x + LEFT_HEADER_MARGIN+1, y +21);
        parent.graphics.fill(255,255,255,75);
        parent.graphics.text(title, x + LEFT_HEADER_MARGIN, y +20);
      }
    } else {
      textFont(bfont,14);
      fill(100);
      noStroke();
      rect(x,y,width,height);
      image(graphics,x,y);
      if (title != null) {
        fill(0,0,0,75);
        text(title, x + LEFT_HEADER_MARGIN+1, y +21);
        fill(255,255,255,75);
        text(title, x + LEFT_HEADER_MARGIN, y +20);
      }      
    }
  }
}


