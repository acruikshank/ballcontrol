import SimpleOpenNI.*;

class Kinect {
	float DEPTH_THRESHOLD = 500; 
	
	SimpleOpenNI context;
	WindowContext depthWindow;
  WindowContext measurementWindow;
	
	Kinect( PApplet parent, WindowContext depthWindow, WindowContext measurementWindow ) {
    this.depthWindow = depthWindow;
    this.measurementWindow = measurementWindow;
	  context = new SimpleOpenNI(parent);
	  context.setMirror(false);

	  if(context.enableDepth() == false)
	  {
	     println("Can't open the depthMap, maybe the camera is not connected!"); 
	     exit();
	     return;
	  }
	}

	PImage depthImage() {
		return context.depthImage();
	}


	Sphere locateBall() {
		ArrayList<ScanLine> scanLines = null;
	  context.update();
	  
	  Sphere sphere = null;
	  PVector[] depthMap = context.depthMapRealWorld();
	  float lastZ = 0;
	  int index =0;
	  PVector point;
	  boolean outside;
	  ScanLine scanLine = null;
	  ScanLineGroup group = null;
	  ArrayList<ScanLineGroup> groups = new ArrayList<ScanLineGroup>();

	  
	  for(int y=0; y < context.depthHeight();y++) {
	    outside = true;
	    lastZ = depthMap[index].z;
	    scanLine = null;
	    for(int x=0;x < context.depthWidth();x++) {
	      point = depthMap[index++];
	      if ( point.z > 0 ) {
	        if ( outside && lastZ - point.z > DEPTH_THRESHOLD ) {
	          if (scanLine == null || point.x - scanLine.endX > 200) {
	          	if (scanLine != null) {
	          		if (group == null || ! group.test(scanLine))
	          			groups.add(group = new ScanLineGroup(scanLine));
	          		else
		          		group.add(scanLine);
	          	}

	            scanLine = new ScanLine( point.x, point.y, point.z );	            
	          } else {
	            scanLine.extend( point.x, point.z );
	          }
	          outside = false;
	        } else if ( ! outside && point.z - lastZ  > DEPTH_THRESHOLD ) {
	          outside = true;
	        } else if ( ! outside ) {
	          scanLine.extend( point.x, point.z);
	        }
	        lastZ = point.z;
	      }
	    }

		  if (scanLine != null) {
	  		if (group == null || ! group.test(scanLine))
	  			groups.add(group = new ScanLineGroup(scanLine));
	  		else
	    		group.add(scanLine);
			}
	  }

		if (groups.size() == 0) {
			return new Sphere(0,0,0,0);
		}

		float maxWeight = 0;
		for ( ScanLineGroup lineGroup : groups ) {
			if ( lineGroup.weight >= maxWeight ) {
				scanLines = lineGroup.scanLines;
				maxWeight = lineGroup.weight;
			}
		}
	  
	  float cx = 0, cy = 0, cz = 0;
	  float r = 0;
	  
	  if ( scanLines.size() > 0 ) {
	    float weight = 0;
	    for ( ScanLine line : scanLines ) {
	      cy += line.y * line.width();
	      cx += (line.startX + (line.endX - line.startX)/2) * line.width();
	      cz += line.averageZ() * line.width();
	      weight += line.width();
	    }
	    cy /= weight;
	    cx /= weight;
	    cz /= weight;
	   
	    r = 0;
	    for ( ScanLine line : scanLines ) {
	      r += Math.sqrt(Math.pow(line.startX-cx,2) + Math.pow(line.y-cy,2));
	      r += Math.sqrt(Math.pow(line.endX-cx,2) + Math.pow(line.y-cy,2));
	    }
	    r /= (scanLines.size() * 2);
	    
	    sphere = new Sphere(cx,cy,cz,r); 
	  }

	  if (depthWindow != null)
	  	renderMeasurement(sphere,scanLines);
	  
	  return sphere;
	}

	void renderMeasurement(Sphere ball, ArrayList<ScanLine> scanLines) {
    PGraphics context = depthWindow.beginContext();
    context.image(kinect.depthImage(),0,0,depthWindow.width,depthWindow.width*3/4.0);
    depthWindow.endContext();

    float s = 2;
    context = measurementWindow.beginContext();
    context.pushMatrix();

    context.translate(measurementWindow.width/2, measurementWindow.height/2);
    context.scale(s,-s);
  
    context.fill(255);
    context.noStroke();
    context.ellipse(0,0,2*ball.r,2*ball.r);
    context.stroke(225,75,25);
    for (ScanLine scanLine : scanLines)
      context.line(scanLine.startX - ball.position.x, scanLine.y - ball.position.y, scanLine.endX - ball.position.x, scanLine.y - ball.position.y );

    context.popMatrix();
    measurementWindow.endContext();
	}
}

class ScanLine {
  float y;
  float startX;
  float endX;
  int samples = 0;
  float totalZ;
  
  ScanLine(float x, float y, float z) {
    this.startX = this.endX = x;
    this.y = y;
    this.totalZ = z;
    samples++;
  }
  
  void extend(float x, float z) {
    endX = x;
    this.totalZ += z;
    samples++;
  }
  
  float width() {
    return endX - startX;
  }
  
  float averageZ() {
    return totalZ / samples;
  }
}

class ScanLineGroup {
	float Y_THRESHOLD = 10;
	float Z_THRESHOLD = 50;
  ScanLine last;
  ArrayList<ScanLine> scanLines = new ArrayList<ScanLine>();
  float weight = 0;
  
  ScanLineGroup(ScanLine first) {
  	add(first);
  }
  
  void add(ScanLine line) {
  	scanLines.add(line);
  	this.last = line;
  	weight += line.endX - line.startX;
  }
  
  boolean test(ScanLine line) {
  	return line.endX > last.startX && line.startX < last.endX && abs(line.y - last.y) < Y_THRESHOLD && abs(line.averageZ() - last.averageZ()) < Z_THRESHOLD;
  }  
}
