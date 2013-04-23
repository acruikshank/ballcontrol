import javax.vecmath.*;

class Sphere {
  float x;
  float y;
  float z;
  float r;
  Sphere( float x, float y, float z, float r ) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.r = r;
  }
  
  float distance(Sphere other) {
    float dx=this.x-other.x, dy = this.y-other.y, dz=this.z-other.z;
    return (float) Math.sqrt(dx*dx + dy*dy + dz*dz); 
  }

  Sphere rotateXZ( float theta ) {
    float rx = x*cos(theta) - z*sin(theta);
    float rz = x*sin(theta) + z*cos(theta);
    return new Sphere( rx, y, rz, r );
  }

  Vector3f vector() {
    return new Vector3f(x,y,z);
  }
}

class Line {
  Vector2f start;
  Vector2f end;
  Line( Vector2f start, Vector2f end) {
    this.start = start;
    this.end = end;
  }

  Vector2f midpoint() {
    Vector2f midpoint = new Vector2f(start);
    Vector2f half = new Vector2f(end);
    half.sub(start);
    half.scale(.5);
    midpoint.add(half);
    return midpoint;
  }

  float length() {
    Vector2f segment = new Vector2f(start);
    segment.sub(end);
    return segment.length();
  }

  Line fromOrigin() {
    Vector2f translated = new Vector2f(end);
    translated.sub(start);
    return new Line( new Vector2f(0,0), translated );
  }

  Line scale( float scale ) {
    Line scaled = fromOrigin();
    scaled.end.scale(scale);
    scaled.start.add(start);
    scaled.end.add(start);
    return scaled;
  }

  Vector2f intersection( Line other ) {
    float x1=start.x,y1=start.y,x2=end.x,y2=end.y,
          x3=other.start.x,y3=other.start.y,x4=other.end.x,y4=other.end.y;
    float det = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    if ( det == 0 )
      return null;
    return new Vector2f( 
      ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4))/det,
      ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4))/det
    );
  }

  Vector2f projectOnto( Vector2f point ) {
    Vector2f projection = new Vector2f(start);
    Vector2f normal = fromOrigin().end;
    normal.normalize();
    Vector2f difference = new Vector2f(point);
    difference.sub(start);
    normal.scale( normal.dot(difference) );
    projection.add(normal);
    return projection;
  }

  float projectDistance( Vector2f point ) {
    Vector2f proj = projectOnto(point);
    proj.sub(point);
    return proj.length();
  }

  Line rotate( float theta ) {
    return new Line( 
      new Vector2f( start.x*cos(theta) - start.y*sin(theta), start.x*sin(theta) + start.y*cos(theta) ),
      new Vector2f( end.x*cos(theta) - end.y*sin(theta), end.x*sin(theta) + end.y*cos(theta) ) );
  }

  Line bisect() {
    Vector2f mid = midpoint();
    Vector2f normal = fromOrigin().end;
    normal.normalize();
    Vector2f sectEnd = new Vector2f(mid);
    sectEnd.add(new Vector2f(-normal.y, normal.x));
    return new Line(mid,sectEnd);
  }
}
