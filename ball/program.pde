import java.util.List;
import java.util.Collections;
import java.util.Comparator;

interface Program {
  boolean step( Sphere position );
}

/*
  CONTROL TEST
  // wait until we have a position on ball (lastLocation != null)
  // mark origin
  // set a goal of r amount of movement from origin
  // each good measurement later
  //   store position
  //   calculate velocity as change in position over time since last good measurement
  //   expected speed = min( max speed, (goal - distance so far) / ease factor )
  //   if velocity < expected speed - tolerance, step up power to motor
  //   if velocity > expected speed + tolerance, step down power to motor
  //   if distance so far > goal, power to motor is 0, stop control test
*/
class CalibrationProgram implements Program {
	int motor = 0;
	Hardware hardware;
	ArrayList<BaselineProgram> programs = new ArrayList<BaselineProgram>();
  ArrayList<Vector2f> eyeBolts = new ArrayList<Vector2f>(3);
	BaselineProgram currentProgram = null;
	float ceiling = 0;
	int cooldown = 0;

  CalibrationProgram( Hardware hardware ) {
  	this.hardware = hardware;
  	currentProgram = new BaselineProgram( motor, hardware);
  }

  boolean step( Sphere position ) {
  	if ( cooldown > 0 ) {
  		cooldown--;
  		return false;
  	}

  	if (motor > 2) {
			for (int i=0; i<3; i++) {
				ceiling += programs.get(i).ceiling;
				eyeBolts.add( programs.get((i+1)%3).baseline.intersection(programs.get((i+2)%3).baseline) );
			}			
			ceiling /= 3.0;

			Vector2f test = new Vector2f(eyeBolts.get(0));
			test.sub(eyeBolts.get(1));
			println("Eyebolt length 1: " + test.length());
			test = new Vector2f(eyeBolts.get(1));
			test.sub(eyeBolts.get(2));
			println("Eyebolt length 2: " + test.length());
			test = new Vector2f(eyeBolts.get(2));
			test.sub(eyeBolts.get(0));
			println("Eyebolt length 3: " + test.length());
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
  	return false;
  }
}

class BaselineProgram implements Program {
  int LINE_SAMPLES = 5;

  int motor = 0;
  Sphere origin = null;
  Hardware hardware;
  Program current = null;
	ArrayList<Sphere> measurements = new ArrayList<Sphere>();
	ArrayList<Sphere> translated = new ArrayList<Sphere>();
  int state = 0;
  int stepCountdown = 5;
  float ax = 0;
  float az = 0;
  float theta = 0;
  Vector2f pointA;
  Vector2f pointB;
  Vector2f midpoint;
  Vector2f arcCenter;
  ArrayList<Vector2f> middle;
  Line bisector2;
  Vector2f center;
  float r = 0;
  Line baseline;
  float ceiling = 0;

	BaselineProgram( int motor, Hardware hardware ) {
		this.motor = motor;
  	println("Starting Calibration of Motor: " + motor);
  	this.hardware = hardware;
  	current = new DistanceProgram( 400, motor, 1, hardware );
	}

  boolean step( Sphere position ) {
  	if ( origin == null )
  		origin = position;

  	switch (state) {
  		case 0:
		    measurements.add(position);
		    if ( current.step( position ) )
    			state = 1;
    		break;
    	case 1:
    		measurements.add(position);
    		if (stepCountdown-- < 1) {
	    		computeXZAngle();
  	  		current = new ReturnProgram( origin, motor, 0, hardware );
					state = 2;    			
    		}
    		break;
    	case 2:
		    if ( current.step( position ) ) {
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

  	ax = az = 0;
  	for (int i=0; i < 2*LINE_SAMPLES; i++) {
  		int sign = (i%2 == 0 ? -1 : 1);
  		float x = samples.get(i).x - samples.get((i+1)%(2*LINE_SAMPLES)).x;
  		float z = samples.get(i).z - samples.get((i+1)%(2*LINE_SAMPLES)).z;
  		float m = (float) Math.sqrt(x*x + z*z);
  		ax += sign * x * m;
  		az += sign * z * m;
  		weight += m;
  	}
  	ax /= weight;
  	az /= weight;
  	theta = atan2(az,ax);

  	for ( Sphere m : measurements )
  		translated.add( m.rotateXZ(-theta) );

  	println("WEIGHT: " + weight);
  	println("THETA: " + theta);
  	println("AX: " + ax);
  	println("AZ: " + az);

  	ArrayList<Vector2f> xys = new ArrayList<Vector2f>(translated.size());
  	float averageZ = 0;
  	for ( Sphere t : translated ) {
  		xys.add( new Vector2f(t.x, t.y) );
  		averageZ += t.z;
  	}
  	averageZ /= translated.size();


  	pointA = averagePoints( xys.subList(0,3) );
  	pointB = averagePoints( xys.subList(xys.size()-3,xys.size()) );
  	Line AB = new Line(pointA,pointB);

  	println("POINT A: " + pointA.x + " "  + pointA.y);
  	println("POINT B: " + pointB.x + " "  + pointB.y);
  	println("AB Length: " + AB.length());
  	midpoint = AB.midpoint();

  	final Line bisector = new Line(pointA,pointB).bisect();
  	println("BISECT start: " + bisector.start.x + " " + bisector.start.y);
  	println("BISECT end: " + bisector.end.x + " " + bisector.end.y);
  	Collections.sort(xys, new Comparator<Vector2f> () {
  		public int compare(Vector2f a, Vector2f b) {
  			float diff = bisector.projectDistance(a) - bisector.projectDistance(b);
  			if (diff == 0) return 0;
  			return diff < 0 ? -1 : 1;
  		}
  	});

  	middle = new ArrayList<Vector2f>(3);
  	for (int i=0; i<3; i++)
  		middle.add( bisector.projectOnto(xys.get(i)) );
  	arcCenter = averagePoints(middle);

  	Line MC = new Line(arcCenter, midpoint);
  	println("MC Length: " + MC.length());

  	float abDist = AB.length();
  	float mcDist = MC.length();
  	r = (abDist*abDist/4.0 + mcDist*mcDist) / (2*mcDist);

  	center = MC.fromOrigin().end;
  	center.normalize();
  	center.scale(r);
  	center.add(arcCenter);
  	println("CENTER: " + center.x + " " + center.y);
  	println("RADIUS: " + r);

  	baseline = new Line(new Vector2f(center.x,averageZ), new Vector2f(center.x,averageZ + 1)).rotate(theta);
  	ceiling = center.y;

  	/*
  	Working with translated:
  	Line fit first 5 point, find closest point on line to point 0 and call it A.
  	Line fit last 5 points, find closest point it to last point and call it B.
  	Find midpoint on AB, M, and the normal from it Mn (assume Mn points away from the circle center).
  	Find the closest 5 points to the right of Mn, and line fit them to line Cr.
		Find the closest 5 points to the left of Mn and line fit them to the line Cl.
		Find the average  of the intersections of Cr with Mn and Cl with Mn. Call it C.
		r = (|AB|^2 + |CM|^2) / (2|CM|)
		center =  C - r|Mn|
		baseLine = (center, center + ((center-C) x AB))
  	*/
  }

  Vector2f averagePoints( List<Vector2f> points) {
  	Vector2f average = new Vector2f(0,0);
  	for (Vector2f point : points) {
  		println( "Adding: " + point.x + " to " + average.x);
  		average.add(point);
  	}
  	average.scale( 1.0 / points.size() );
  	println( "after scale: " + average.x);
  	return average;
  }

  /* given 5 points ordered along the line, find */
  /*
  Vector3f lineFit5( ArrayList<Vector3f> points ) {
  	int[] indices = {2,3,1,4,0,3,1,2};
  	Vector3f line = Vector3f(0,0,0);
  	for (int index=0; index < indices.length - 1; index++) {
  		Vector3f next = points.get(index+1).sub(points.get(index));
  		line.add( next.scale( (i%2==0?1:-1) * next.length() ) );
  	}
  	return line.normalize();
  }
  */
}


abstract class SingleAxisMotionProgram implements Program {
  float MAX_SPEED = 125;
  float TOLERANCE = 50;
  int MOTOR_INCREMENT = 2;

  float EASE_OFFSET = .5;
  float EASE_START = .75;

  Sphere lastPosition = new Sphere(0,0,0,0);
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

  void move( float fraction, Sphere position ) {
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
    println( fraction + " " + expectedVelocity + " " + velocity + " " + motorPower );
    
    lastPosition = position;
    lastMeasurement = new Date();
    return;
  }
}

class DistanceProgram extends SingleAxisMotionProgram {
  Sphere origin = null;
  float goal = 0;
  
  DistanceProgram( float goal, int motor, int direction, Hardware hardware ) {
  	super( motor, direction, hardware );
    println("Moving " + motor + " " + goal + " units");
    this.goal = goal;
  }
  
  boolean step( Sphere position ) {
  	if ( origin == null )
  		origin = position;

    float totalDistance = position.distance(origin);
    if ( totalDistance >= goal) {
      hardware.sendCommand( motor, 0, 0 );
      println("End move");
      return true;
    }

    move( totalDistance / goal, position );
    return false;
  }
}

class ReturnProgram extends SingleAxisMotionProgram {
  float TOLERANCE = 50;

  Sphere origin = null;
  Sphere destination = null;

  ReturnProgram( Sphere destination, int motor, int direction, Hardware hardware ) {
  	super( motor, direction, hardware );
    println("Moving " + motor + " to origin");
    this.destination = destination;
  }
  
  boolean step( Sphere position ) {
  	if ( origin == null )
  		origin = position;

    float totalDistance = position.distance(destination);
    if ( totalDistance <= TOLERANCE) {
      hardware.sendCommand( motor, 0, 0 );
      println("End move");
      return true;
    }

    move( 1 - totalDistance / origin.distance(destination), position );
    return false;
  }
}