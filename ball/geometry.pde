
class Pt {
  float x;
  float y;
  float z;
  public Pt( float x, float y, float z ) {
    this.x=x; this.y=y; this.z=z;
  }

  Pt scale(float s) { return new Pt(s*x, s*y, s*z); }
  Pt add(Pt b) { return new Pt(x+b.x,y+b.y,z+b.z); }
  Pt sub(Pt b) { return new Pt(x-b.x,y-b.y,z-b.z); }
  float dot(Pt b) { return x*b.x + y*b.y + z*b.z; }
  float mag() { return sqrt(x*x+y*y+z*z); }
  float distance(Pt other) { return sub(other).mag(); }
  Pt rotateXZ(float theta) { return new Pt(x*cos(theta) - z*sin(theta), y, x*sin(theta) + z*cos(theta)); }
}

class Sphere {
  Pt position;
  float r;
  Sphere( float x, float y, float z, float r ) {
    this( new Pt(x,y,z), r );
  }
  Sphere( Pt p, float r ) {
    position = p;
    this.r = r;
  }
}

class Line {
  Pt start;
  Pt end;
  Line( Pt start, Pt end) {
    this.start = start;
    this.end = end;
  }

  Pt midpoint() { return start .add ( (end .sub (start)).scale(.5) ); }
  float length() { return start.distance(end); }
  Line fromOrigin() { return new Line(new Pt(0f,0f,0f), end.sub(start)); }
  Line scale( float scale ) { return new Line(start, ( (end .sub (start)).scale(scale) ) .add (start) ); }

  /*
    Finds the intersection of the projections of two lines onto the XY plane.
    The resulting point will be given the z value of this line's start.
    If the projection of the two lines are parallel, this function returns null.
   */
  Pt intersectionXY( Line other ) {
    float x1=start.x,y1=start.y,x2=end.x,y2=end.y,
          x3=other.start.x,y3=other.start.y,x4=other.end.x,y4=other.end.y;
    float det = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4);
    if ( det == 0 )
      return null;
    return new Pt( 
      ((x1*y2 - y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4))/det,
      ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4))/det,
      start.z
    );
  }

  Line normal() {
    Line oline = fromOrigin();
    return new Line(oline.start, oline.end.scale(1/oline.end.mag()) );
  }

  Pt projectOnto( Pt point ) {
    Pt norm = normal().end;
    return start .add ( norm.scale( norm.dot( point .sub (start) ) ) );
  }

  float projectDistance( Pt point ) {
    return projectOnto(point) .distance (point);
  }

  Line rotateXZ( float theta ) {
    return new Line( start.rotateXZ(theta), end.rotateXZ(theta) );
  }

  Line extend( float scale ) {
    return new Line( start, normal().end.scale(scale).add(start) );
  }

  Line bisectXY() {
    Pt mid = midpoint();
    Pt norm = normal().end;
    return new Line(mid, mid .add (new Pt(-norm.y, norm.x, norm.z)));
  }
}
