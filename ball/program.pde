import java.util.List;
import java.util.Collections;
import java.util.Comparator;

interface Program {
  boolean step( Sphere position );
}

float MEASURED_CEILING_Y = 711;

/*
  CALIBRATION
  The goal is to determine the location of each of the eyebolts using measurements
  of the ball's path when only one motor is activated. Geometry tells us this path
  will be a circle centered on and orthogonal to the line between the two eybolts
  opposite of the eyebolt through with the current motor's cord is passing.

  Since we don't yet have any knowlege of the boundaries of the experiment, take
  a conservative approach of slowly increasing power to one of the motors until
  we detect movement in the ball. Try to maintain this velocity until the ball has
  traveled up a small distance. Then reverse the direction and move the ball until
  it is sufficiently close to its origin. Use the upward path measurements to
  determine the center.

  We start by rotating the measurements onto the xy plane estimating the center of 
  the circle geometrically, then use gradient descent to minimize the mean squared 
  error. It turns out that we can't get consistent results if we try to simultaneously
  solve for the x and y of the center and the radius, so we start by callibrating the
  Kinect so that its yz plane is parallel to the ceiling and then measuring the y
  distance between its sensor and the y value of an eyebolt (i.e. a little less than
  the vertical distance between the sensor and the ceiling). Once we have the xy
  coordinates of the center we rotate it back onto its original plane and take its
  normal to create a line.

  Repeating this procedure for all three motors gives us three lines and the
  intersections of these lines are our estimates for the eyebolt locations in
  the Kinect's 3D coordinates.
*/
class CalibrationProgram implements Program {

	int motor = 0;
	Hardware hardware;
	ArrayList<BaselineProgram> programs = new ArrayList<BaselineProgram>();
  ArrayList<Pt> eyeBolts = new ArrayList<Pt>(3);
	BaselineProgram currentProgram = null;
	float ceiling = 0;
	int cooldown = 0;
  WindowContext orthoXY;
  WindowContext orthoYZ;
  WindowContext orthoXZ;
  WindowContext solution;
  WindowContext mainWindow;

  CalibrationProgram( Hardware hardware, WindowContext window ) {
  	this.hardware = hardware;
    mainWindow = window;
  	currentProgram = new BaselineProgram( motor, hardware);

    if (window != null) {
      orthoXY = new WindowContext("Ortho XY", 0, 0, window.width/2, window.height/2, window);
      orthoYZ = new WindowContext("Ortho YZ", window.width/2, 0, window.width/2, window.height/2, window);
      orthoXZ = new WindowContext("Ortho XZ", 0, window.height/2, window.width/2, window.height/2, window);
      solution = new WindowContext("Solution", window.width/2, window.height/2, window.width/2, window.height/2, window);
    }
  }

  boolean step( Sphere position ) {
  	if ( cooldown > 0 ) {
  		cooldown--;
  		return false;
  	}

  	if (motor > 2) {
			for (int i=0; i<3; i++) {
				ceiling += programs.get(i).ceiling;
				eyeBolts.add( programs.get((i+1)%3).baseline.intersectionXZ(programs.get((i+2)%3).baseline) );
			}			
			ceiling /= 3.0;

      if (mainWindow != null)
        renderCalculations();

  		return true;
  	}

  	if (currentProgram == null)
  		currentProgram = new BaselineProgram( motor, hardware );

  	if (currentProgram.step(position)) {
  		motor++;
  		programs.add(currentProgram);
  		currentProgram = null;
  		cooldown = 10;
  	}

    if (mainWindow != null)
      renderCalculations();

  	return false;
  }

  void renderCalculations() {
    float s = .25;
    float Z_OFFSET = 1200;

    mainWindow.beginContext();
    PGraphics xy = orthoXY.beginContext();
    initGraph(orthoXY, xy,s,true,true);
    PGraphics yz = orthoYZ.beginContext();
    initGraph(orthoYZ, yz,s,true,false);
    PGraphics xz = orthoXZ.beginContext();
    initGraph(orthoXZ, xz,s,false,true);
    PGraphics sol = solution.beginContext();
    
    float mRadius = 10;

    if (eyeBolts.size() == 3) {
      solution.writeLine( String.format("Eyebolt 1 x:%7.1f    y:%7.1f    z:%7.1f", eyeBolts.get(0).x, eyeBolts.get(0).y, eyeBolts.get(0).z  ) );
      solution.writeLine( String.format("Eyebolt 2 x:%7.1f    y:%7.1f    z:%7.1f", eyeBolts.get(1).x, eyeBolts.get(1).y, eyeBolts.get(1).z  ) );
      solution.writeLine( String.format("Eyebolt 3 x:%7.1f    y:%7.1f    z:%7.1f", eyeBolts.get(2).x, eyeBolts.get(2).y, eyeBolts.get(2).z  ) );
      solution.writeLine( String.format("Baseline 1-2 Length: %8.1f", new Line(eyeBolts.get(0),eyeBolts.get(1)).length() ) );
      solution.writeLine( String.format("Baseline 2-3 Length: %8.1f", new Line(eyeBolts.get(1),eyeBolts.get(2)).length() ) );
      solution.writeLine( String.format("Baseline 3-1 Length: %8.1f", new Line(eyeBolts.get(2),eyeBolts.get(0)).length() ) );
    }

    initGraph(solution, sol,s/2,false,true);

    for (BaselineProgram baselineProgram : calibrationProgram.programs) {

      xy.fill(150,150,200);
      yz.fill(150,150,200);
      xz.fill(150,150,200);
      for ( Sphere measurement : baselineProgram.measurements ) {
        xy.ellipse(measurement.position.x,measurement.position.y,mRadius,mRadius);
        yz.ellipse(Z_OFFSET-measurement.position.z,measurement.position.y,mRadius,mRadius);
        xz.ellipse(measurement.position.x,Z_OFFSET-measurement.position.z,mRadius,mRadius);
      }

      if ( baselineProgram.translated.size() > 0 ) {


        float offset = baselineProgram.translated.get(0).x;
        xy.fill(150,255,150);
        xy.ellipse(baselineProgram.pointA.x-offset,baselineProgram.pointA.y,2*mRadius,2*mRadius);      
        xy.ellipse(baselineProgram.pointB.x-offset,baselineProgram.pointB.y,2*mRadius,2*mRadius);      

        xy.fill(150,150,255);
        xy.ellipse(baselineProgram.midpoint.x-offset,baselineProgram.midpoint.y,2*mRadius,2*mRadius);
        xy.ellipse(baselineProgram.arcCenter.x-offset,baselineProgram.arcCenter.y,2*mRadius,2*mRadius);
        for ( Pt measurement : baselineProgram.middle )
          xy.ellipse(measurement.x-offset,measurement.y,mRadius,mRadius);

        xy.fill(255,150,150);
        for ( Pt measurement : baselineProgram.translated )
          xy.ellipse(measurement.x-offset,measurement.y,mRadius,mRadius);

        xy.noFill();
        xy.stroke(255,255,100);
        xy.line(baselineProgram.arcCenter.x-offset,baselineProgram.arcCenter.y,baselineProgram.center.x-offset,baselineProgram.center.y);
        xy.ellipse(baselineProgram.center.x-offset,baselineProgram.center.y,baselineProgram.r,baselineProgram.r);
        xy.stroke(100,100,100);
      }

      if ( baselineProgram.baseline != null ) {
        sol.stroke(255,255,100);
        sol.strokeWeight(5);
        Line scaled = baselineProgram.baseline.scale(10000);
        sol.line( scaled.start.x, Z_OFFSET - scaled.start.z, scaled.end.x, Z_OFFSET - scaled.end.z );
        scaled = baselineProgram.baseline.scale(-10000);
        sol.line( scaled.start.x, Z_OFFSET - scaled.start.z, scaled.end.x, Z_OFFSET - scaled.end.z );
        sol.strokeWeight(1);
      }

    }
  
    for (Pt eyeBolt : calibrationProgram.eyeBolts) {
      sol.fill(100);
      sol.stroke(255,255,0);
      sol.strokeWeight(2);
      sol.ellipse(eyeBolt.x,Z_OFFSET-eyeBolt.z,2*mRadius,2*mRadius);
    }

    xy.popMatrix();
    yz.popMatrix();
    xz.popMatrix();
    sol.popMatrix();
    orthoXY.endContext();
    orthoYZ.endContext();
    orthoXZ.endContext();
    solution.endContext();
    mainWindow.endContext();

  }

  void initGraph( WindowContext window, PGraphics context, float scale, boolean xaxis, boolean yaxis ) {
    context.ellipseMode(RADIUS);
    context.stroke(75);

    context.pushMatrix();
    if (xaxis)
      context.line(0,window.height/2,window.width,window.height/2);
    if (yaxis)
      context.line(orthoYZ.width/2,0,orthoYZ.width/2,orthoYZ.height);
    
    context.translate(window.width/2, window.height/2);
    context.scale(scale,-scale);
  }
}

class BaselineProgram implements Program {
  int LINE_SAMPLES = 5;

  int motor = 0;
  Pt origin = null;
  Hardware hardware;
  Program current = null;
	ArrayList<Sphere> measurements = new ArrayList<Sphere>();
	ArrayList<Pt> translated = new ArrayList<Pt>();
  int state = 0;
  int stepCountdown = 5;
  float ax = 0;
  float az = 0;
  float theta = 0;
  Pt pointA;
  Pt pointB;
  Pt midpoint;
  Pt arcCenter;
  ArrayList<Pt> middle;
  Line bisector2;
  Pt center;
  float r = 0;
  Line baseline;
  float ceiling = 0;

	BaselineProgram( int motor, Hardware hardware ) {
		this.motor = motor;
  	println("Starting Calibration of Motor: " + motor);
  	this.hardware = hardware;
  	current = new DistanceProgram( 400, motor, 1, hardware );
	}

  boolean step( Sphere measurement ) {
  	if ( origin == null )
  		origin = measurement.position;

  	switch (state) {
  		case 0:
		    measurements.add(measurement);
		    if ( current.step( measurement ) )
    			state = 1;
    		break;
    	case 1:
    		measurements.add(measurement);
    		if (stepCountdown-- < 1) {
	    		computeXZAngle();
  	  		current = new ReturnProgram( origin, motor, 0, hardware );
					state = 2;    			
    		}
    		break;
    	case 2:
		    if ( current.step( measurement ) ) {
    			state = 3;
    			return true;
    		}
  	}
    return false;
  }

  void computeXZAngle() {
  	float weight = 0, count = measurements.size();
  	ArrayList<Sphere> samples = new ArrayList<Sphere>(LINE_SAMPLES*2);

  	for (int i=0; i < 2*LINE_SAMPLES; i++)
  		samples.add( measurements.get(int(i%2 == 0 ? random(0,count/2) :random(count/2,count))) );

    // Expect projection onto xz plane to be a line.
    // Analyze samples to find average angle, theta.
  	ax = az = 0;
  	for (int i=0; i < 2*LINE_SAMPLES; i++) {
      Pt current = samples.get(i).position;
      Pt next = samples.get((i+1)%(2*LINE_SAMPLES)).position;
  		int sign = (i%2 == 0 ? -1 : 1);
  		float x = current.x - next.x;
  		float z = current.z - next.z;
  		float m = sqrt(x*x + z*z);
  		ax += sign * x * m;
  		az += sign * z * m;
  		weight += m;
  	}
  	ax /= weight;
  	az /= weight;
  	theta = atan2(az,ax);

    float averageZ = 0;
  	for ( Sphere m : measurements ) {
      Pt t = m.position.rotateXZ(-theta);
  		translated.add( t );
      averageZ += t.z;
    }
    averageZ /= translated.size();

  	println("WEIGHT: " + weight);
  	println("THETA: " + theta);
  	println("AX: " + ax);
  	println("AZ: " + az);
    println("AVERAGE Z: " + averageZ);

    // Points have now been rotated onto a plane orthogonal to the z axis.
    // We expect the points to lie on an arc of a circle on this plane.
    // Compute the midpoint, M, of the line connecting the two ends of the
    // arc, AB. Let a be the distance from one end of the arc to M, and b
    // be the distance from the center of the arc, C, to M (note that point C
    // will lie on the bisection of AB). The radius of the circle is given 
    // by r=(a^2+b^2)/2b and the center of the circle lies on the line CM at 
    // a distance r from C.

    // This algorithm works, but it is very sensitive to measurement error
    // of locations of the points at the center of the arc. It also discards
    // the information from all points not at the center or ends. A
    // least-squares approach would work better here.
  	pointA = averagePointsXY( translated.subList(0,3), averageZ );
  	pointB = averagePointsXY( translated.subList(translated.size()-3,translated.size()), averageZ );
  	Line AB = new Line(pointA,pointB);

  	println("POINT A: " + pointA.x + " "  + pointA.y);
  	println("POINT B: " + pointB.x + " "  + pointB.y);
  	println("AB Length: " + AB.length());
  	midpoint = AB.midpoint();

  	final Line bisector = new Line(pointA,pointB).bisectXY();
  	println("BISECT start: " + bisector.start.x + " " + bisector.start.y);
  	println("BISECT end: " + bisector.end.x + " " + bisector.end.y);

    // sort translated to find points nearest the midpoint bisector
  	Collections.sort(translated, new Comparator<Pt> () {
  		public int compare(Pt a, Pt b) {
  			float diff = bisector.projectDistance(a) - bisector.projectDistance(b);
  			if (diff == 0) return 0;
  			return diff < 0 ? -1 : 1;
  		}
  	});

  	middle = new ArrayList<Pt>(3);
  	for (int i=0; i<3; i++)
  		middle.add( bisector.projectOnto(translated.get(i)) );
  	arcCenter = averagePointsXY(middle, averageZ);

  	Line MC = new Line(arcCenter, midpoint);
  	println("MC Length: " + MC.length());

  	float abDist = AB.length();
  	float mcDist = MC.length();
  	r = (abDist*abDist/4.0 + mcDist*mcDist) / (2*mcDist);

    center = MC.extend(r).end;

    /*
    List<CircleEstimate> estimates = gradientDescent(translated, new CircleEstimate(center.x, center.y,r), 500, .003);
    */
    List<CircleEstimate> estimates = gradientDescentWithFixedY(translated, new CircleEstimate(center.x, MEASURED_CEILING_Y, r), 500, .003);
    for (CircleEstimate estimate: estimates)
      println(String.format("estimate x: %6.1f  y: %6.1f  r: %6.1f     mse: %2.4f", estimate.x, estimate.y, estimate.r, meanSquareError(translated,estimate) ));

    CircleEstimate bestGuess = estimates.get(estimates.size()-1);
    center = new Pt(bestGuess.x, bestGuess.y, center.z);
  	println("CENTER: " + center.x + " " + center.y);
  	println("RADIUS: " + bestGuess.r);

  	baseline = new Line(center, center.add(new Pt(0,0,1)) ).rotateXZ(theta);
  }

  Pt averagePointsXY( List<Pt> points, float z) {
  	Pt average = new Pt(0,0,0);
  	for (Pt point : points)
  		average = average.add(point);
  	average = average.scale( 1.0 / points.size() );
    average.z = z;
    return average;
  }

  List<CircleEstimate> gradientDescent(List<Pt> samples, CircleEstimate estimate, int iterations, float rate) {    
    List<CircleEstimate> estimates = new ArrayList<CircleEstimate>();
    estimates.add(estimate);
    for (int i=0; i<iterations; i++) {
      float x = estimate.x, y = estimate.y, r = estimate.r;
      for (Pt sample : samples) {
        float dx = estimate.x - sample.x, 
              dy = estimate.y - sample.y, 
              distance = (float) Math.sqrt(dx*dx+dy*dy),
              error = estimate.r - distance;
        x += rate * 2*dx*error / distance;
        y += rate * 2*dy*error / distance;
        r -= rate*2*error;
      }
      estimate = new CircleEstimate(x,y,r);
      estimates.add(estimate);
    }
    return estimates;
  }

  List<CircleEstimate> gradientDescentWithFixedY(List<Pt> samples, CircleEstimate estimate, int iterations, float rate) {    
    List<CircleEstimate> estimates = new ArrayList<CircleEstimate>();
    estimates.add(estimate);
    for (int i=0; i<iterations; i++) {
      float x = estimate.x, y = estimate.y, r = estimate.r;
      for (Pt sample : samples) {
        float dx = estimate.x - sample.x, 
              dy = estimate.y - sample.y, 
              distance = (float) Math.sqrt(dx*dx+dy*dy),
              error = estimate.r - distance;
        x += rate * 2*dx*error / distance;
        r -= rate*2*error;
      }
      estimate = new CircleEstimate(x,y,r);
      estimates.add(estimate);
    }
    return estimates;
  }

  float meanSquareError(List<Pt> samples, CircleEstimate estimate) {
    float cost = 0;
    for (Pt sample : samples) {
      float dx = estimate.x - sample.x, 
          dy = estimate.y - sample.y, 
          distance = (float) Math.sqrt(dx*dx+dy*dy),
          error = estimate.r - distance;
      cost += error*error;
    }
    return cost / samples.size();
  }

  class CircleEstimate {
    float x;
    float y;
    float r;
    CircleEstimate(float x, float y, float r) {
      this.x = x;
      this.y = y;
      this.r = r;
    }
  }
}


abstract class SingleAxisMotionProgram implements Program {
  float MAX_SPEED = 125;
  float TOLERANCE = 50;
  int MOTOR_INCREMENT = 2;

  float EASE_OFFSET = .5;
  float EASE_START = .75;

  Pt lastPosition = new Pt(0,0,0);
  Date lastMeasurement = null;
  int motorPower = 0;

  int motor = 0;
  int direction = 1;
  Hardware hardware = null;

  SingleAxisMotionProgram( int motor, int direction, Hardware hardware ) {
  	this.motor = motor;
  	this.direction = direction;
  	this.hardware = hardware;
  }

  void move( float fraction, Pt position ) {
    if ( lastMeasurement == null ) {
      lastPosition = position;
      lastMeasurement = new Date();
      return;
    }
    
    float expectedVelocity = MAX_SPEED * EASE_OFFSET + (MAX_SPEED - EASE_OFFSET) * (1 - fraction) / (1 - EASE_START);
    expectedVelocity = min(MAX_SPEED,max(0, expectedVelocity)); 
    float velocity = 1000 * position.distance(lastPosition) / (new Date().getTime() - lastMeasurement.getTime());
    if (velocity > expectedVelocity + TOLERANCE) {
      motorPower = max(0,min(255,motorPower - MOTOR_INCREMENT));
      hardware.sendCommand( motor, direction, motorPower );
    } else if (velocity < expectedVelocity - TOLERANCE) {
      motorPower = max(0,min(255,motorPower + MOTOR_INCREMENT));
      hardware.sendCommand( motor, direction, motorPower );
    }
//    println( fraction + " " + expectedVelocity + " " + velocity + " " + motorPower );
    
    lastPosition = position;
    lastMeasurement = new Date();
    return;
  }
}

class DistanceProgram extends SingleAxisMotionProgram {
  Pt origin = null;
  float goal = 0;
  
  DistanceProgram( float goal, int motor, int direction, Hardware hardware ) {
  	super( motor, direction, hardware );
    println("Moving " + motor + " " + goal + " units");
    this.goal = goal;
  }
  
  boolean step( Sphere measurement ) {
  	if ( origin == null )
  		origin = measurement.position;

    float totalDistance = measurement.position.distance(origin);
    if ( totalDistance >= goal) {
      hardware.sendCommand( motor, 0, 0 );
      println("End move");
      return true;
    }

    move( totalDistance / goal, measurement.position );
    return false;
  }
}

class ReturnProgram extends SingleAxisMotionProgram {
  float TOLERANCE = 50;

  Pt origin = null;
  Pt destination = null;

  ReturnProgram( Pt destination, int motor, int direction, Hardware hardware ) {
  	super( motor, direction, hardware );
    println("Moving " + motor + " to origin");
    this.destination = destination;
  }
  
  boolean step( Sphere measurement ) {
  	if ( origin == null )
  		origin = measurement.position;

    float totalDistance = measurement.position.distance(destination);
    if ( totalDistance <= TOLERANCE) {
      hardware.sendCommand( motor, 0, 0 );
      println("End move");
      return true;
    }

    move( 1 - totalDistance / origin.distance(destination), measurement.position );
    return false;
  }
}