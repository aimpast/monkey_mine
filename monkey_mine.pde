

import javax.vecmath.Vector3f;
/*import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.GhostObject;
import com.bulletphysics.collision.dispatch.GhostPairCallback;*/

import peasy.*;
import bRigid.*;

PeasyCam cam;

BPhysics physics;

BBox box1;
BBox box2;
BJointHinge hinge;
BJointHinge hinge2;

public void setup() {
  size(640,480,P3D);
  frameRate(60);
  
  cam = new PeasyCam(this, 600);
  
  Vector3f min = new Vector3f(-1200, -2500, -1200);
  Vector3f max = new Vector3f(1200, 2500, 1200);
  
  physics = new BPhysics(min, max);
  
  physics.world.setGravity(new Vector3f(0,500, 0));
  
  Vector3f pos1 = new Vector3f(0,-100,0);
  Vector3f pos2 = new Vector3f(0,-60,0);
  Vector3f pos4 = new Vector3f(0, 0,0);
  
  box1 = new BBox(this,100,500, 10,500);
  box2 = new BBox(this,  1, 50, 50, 50);
  
  BObject b1 = new BObject(this,100,box1,pos1,true);
  BObject b2 = new BObject(this,100,box2,pos2,true);
  BObject b3 = new BObject(this,  0,box2,pos4,true);
  
  //b1.setPosition(pos1);
  //b2.setPosition(pos2);
  
  Vector3f dist = b2.rigidBody.getCenterOfMassPosition(new Vector3f());
  Vector3f t    = b1.rigidBody.getCenterOfMassPosition(new Vector3f());
  dist.sub(t);
  Vector3f pivA = (Vector3f) dist.clone();
  pivA.scale(-.5f);
  Vector3f pivB = (Vector3f) dist.clone();
  pivB.scale(.5f);
  
  Vector3f axisInA = new Vector3f(0, 0, 1);
  axisInA.normalize();
  Vector3f axisInB = (Vector3f) axisInA.clone();
  
  hinge = new BJointHinge(b2,b1,pivA,pivB,axisInA,axisInB);
  physics.addJoint(hinge);
  
  
  
  
  Vector3f dist2 = b3.rigidBody.getCenterOfMassPosition(new Vector3f());
  Vector3f t2     = b2.rigidBody.getCenterOfMassPosition(new Vector3f());
  dist2.sub(t2);
  Vector3f pivA2 = (Vector3f) dist2.clone();
  pivA2.scale(-.5f);
  Vector3f pivB2 = (Vector3f) dist2.clone();
  pivB2.scale(.5f);
  
  Vector3f axisInA2 = new Vector3f(1, 0, 0);
  axisInA.normalize();
  Vector3f axisInB2 = (Vector3f) axisInA2.clone();
  
  hinge2 = new BJointHinge(b3,b2,pivA2,pivB2,axisInA2,axisInB2);
  physics.addJoint(hinge2);
  

  
  Vector3f pos3 = new Vector3f(0,0,0);
  BSphere sphere = new BSphere(this,10,20,pos3,true);
  sphere.setPosition(new Vector3f(200,-500,200));
  
  //BObject s = new BObject(this,100,sphere,pos3,true);
  physics.addBody(b1);
  physics.addBody(b2);
  physics.addBody(b3);
  physics.addBody(sphere);

  
  hinge.setLimit(-.5f, .5f);
  hinge2.setLimit(-.5f, .5f);
  //s.setPosition(pos3);
}  

public void draw() {
  
  background(0);
  lights();
  
  if (hinge.getHingeAngle() < 0) {
    
    hinge.enableAngularMotor(true, -hinge.getHingeAngle(), 20000f);
  }
  else if (hinge.getHingeAngle() > 0) {
    hinge.enableAngularMotor(true, -hinge.getHingeAngle(), 20000f);
  }
  
  if (hinge2.getHingeAngle() < 0) {
    hinge2.enableAngularMotor(true, -hinge2.getHingeAngle(), 20000f);
  }
  else if (hinge2.getHingeAngle() > 0){
    hinge2.enableAngularMotor(true, -hinge2.getHingeAngle(), 20000f);
  }
  
  physics.update();
  physics.display();
}
