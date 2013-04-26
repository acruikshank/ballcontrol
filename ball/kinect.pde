import SimpleOpenNI.*;

class Kinect {
	float DEPTH_THRESHOLD = 500; 
  float MIN_RADIUS = 10.0;
  float MAX_RADIUS = 30.0;
	
	SimpleOpenNI context;
	WindowContext depthWindow;
  WindowContext measurementWindow;
  WindowContext statsWindow;
  int MEASUREMENT_BUFFER = 30;
  Sphere[] measurements = new Sphere[MEASUREMENT_BUFFER];
  int measurementIndex = 0;

	
	Kinect( PApplet parent, WindowContext depthWindow, WindowContext measurementWindow, WindowContext statsWindow ) {
    this.depthWindow = depthWindow;
    this.measurementWindow = measurementWindow;
    this.statsWindow = statsWindow;
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

		if (groups.size() == 0)
			return null;

		float maxWeight = 0;
    ScanLineGroup biggestGroup = null;
		for ( ScanLineGroup lineGroup : groups ) {
			if ( lineGroup.weight >= maxWeight ) {
        biggestGroup = group;
				maxWeight = lineGroup.weight;
			}
		}

    scanLines = biggestGroup.maxSquareVerticalConvolution().scanLines;
    //scanLines = biggestGroup.scanLines;
	  
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

      if (weight == 0)
        return null;

	    cy /= weight;
	    cx /= weight;
	    cz /= weight;
	   
	    r = 0;
	    for ( ScanLine line : scanLines ) {
	      r += Math.sqrt(Math.pow(line.startX-cx,2) + Math.pow(line.y-cy,2));
	      r += Math.sqrt(Math.pow(line.endX-cx,2) + Math.pow(line.y-cy,2));
	    }
	    r /= (scanLines.size() * 2);

      if ( r > MAX_RADIUS || r < MIN_RADIUS )
        return null;
	    
	    sphere = new Sphere(cx,cy,cz,r); 

  	  if (depthWindow != null)
  	  	renderMeasurement(sphere, biggestGroup.scanLines, scanLines);
    }
	  
	  return sphere;
	}

	void renderMeasurement(Sphere ball, ArrayList<ScanLine> scanLines, ArrayList<ScanLine> optimizedLines) {
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

    context.stroke(25,75,225,75);
    for (ScanLine scanLine : scanLines)
      context.line(scanLine.startX - ball.position.x, scanLine.y - ball.position.y, scanLine.endX - ball.position.x, scanLine.y - ball.position.y );

    context.stroke(225,75,25,255);
    for (ScanLine scanLine : optimizedLines)
      context.line(scanLine.startX - ball.position.x, scanLine.y - ball.position.y, scanLine.endX - ball.position.x, scanLine.y - ball.position.y );

    context.popMatrix();
    measurementWindow.endContext();

    context = statsWindow.beginContext();

    measurements[measurementIndex] = ball;
    measurementIndex = (measurementIndex+1) % MEASUREMENT_BUFFER;

    StandardDeviation stdX = new StandardDeviation();
    StandardDeviation stdY = new StandardDeviation();
    StandardDeviation stdZ = new StandardDeviation();
    StandardDeviation stdR = new StandardDeviation();
    for (Sphere m : measurements) {
      if (m != null) {
        stdX.add(m.position.x);
        stdY.add(m.position.y);
        stdZ.add(m.position.z);
        stdR.add(m.r);
      }
    }

    statsWindow.writeLine( String.format("X: %8.1f       std: %4.2f", ball.position.x, stdX.compute()) );
    statsWindow.writeLine( String.format("Y: %8.1f       std: %4.2f", ball.position.y, stdY.compute()) );
    statsWindow.writeLine( String.format("Z: %8.1f       std: %4.2f", ball.position.z, stdZ.compute()) );
    statsWindow.writeLine( String.format("R: %8.1f       std: %4.2f", ball.r, stdR.compute()) );
    statsWindow.endContext();
	}
}

class StandardDeviation {
  ArrayList<Float> samples = new ArrayList<Float>();
  float total = 0;
  void add(float sample) {
    samples.add(sample);
    total += sample;
  }

  float compute() {
    float mean = total/samples.size();
    float errorSum = 0;
    for (float sample : samples)
      errorSum += (sample-mean)*(sample-mean);
    return (float) Math.sqrt(errorSum/samples.size());
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

  ScanLineGroup(List<ScanLine> lines) {
    for (ScanLine line : lines) add(line);
  }
  
  void add(ScanLine line) {
  	scanLines.add(line);
  	this.last = line;
  	weight += line.endX - line.startX;
  }

  float xRange() {
    float minX = 100000000;
    float maxX = -1000000000;
    for (ScanLine line : scanLines) {
      if (line.startX < minX) minX = line.startX;
      if (line.endX > maxX) maxX = line.endX;
    }
    return maxX - minX;
  }

  /* The kinect sometimes picks up the weight at the bottom of the ball and the string at the top.
     We assume the ball is wider than these artifacts and spherical (of course), so we take the
     maximum and minimum x values from the lines in this group and assume that range gives us the
     height of the ball. We then convolve through the group using that
     range to find the vertical section of that height covering the largest area, which should be
     the ball without all the stuff attached.
   */
  ScanLineGroup maxSquareVerticalConvolution() {
    float range = xRange();
    int startIndex = 0;
    int length = 0;
    float convolution = 0;
    float maxConvolution = 0;
    int maxStartIndex = 0;
    int maxLength = 0;

    for (ScanLine line : scanLines) {
      convolution += line.width();
      length += 1;
      
      while ( startIndex < scanLines.size() &&  abs(line.y - scanLines.get(startIndex).y) > range ) {
        convolution -= scanLines.get(startIndex).width();
        startIndex += 1;
        length -= 1;
      }

      if (convolution > maxConvolution) {
        maxConvolution = convolution;
        maxStartIndex = startIndex;
        maxLength = length;
      }
    }

    return new ScanLineGroup( scanLines.subList(maxStartIndex,maxStartIndex+maxLength) );
  }
  
  boolean test(ScanLine line) {
  	return line.endX > last.startX && line.startX < last.endX && abs(line.y - last.y) < Y_THRESHOLD && abs(line.averageZ() - last.averageZ()) < Z_THRESHOLD;
  }  
}
