

import javax.vecmath.Vector3f;
import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.linearmath.Transform;
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


int[][] map = {
  {1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,1},
  {1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1}
};

public void setup() {
  size(640,480,P3D);
  frameRate(60);
  
  cam = new PeasyCam(this, 0,-400,0, 1000);
  
  Vector3f min = new Vector3f(-1200, -2500, -1200);
  Vector3f max = new Vector3f(1200, 2500, 1200);
  
  physics = new BPhysics(min, max);
  
  physics.world.setGravity(new Vector3f(0,500, 0));
  
  Vector3f pos1 = new Vector3f(0,-100,0);
  Vector3f pos2 = new Vector3f(0,-60,0);
  Vector3f pos4 = new Vector3f(0, 0,0);
  
  box1 = new BBox(this,100,1000, 10,1000);
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
  BSphere sphere = new BSphere(this,1,20,pos3,true);
  sphere.setPosition(new Vector3f(50,-500,50));
  
  //BObject s = new BObject(this,100,sphere,pos3,true);
  physics.addBody(b1);
  physics.addBody(b2);
  physics.addBody(b3);
  physics.addBody(sphere);

  
  hinge.setLimit(-.5f, .5f);
  hinge2.setLimit(-.5f, .5f);
  //s.setPosition(pos3);
  
  for (int j=0;j<20;j++) {
    for (int i=0;i<20;i++) {
      if (map[j][i] == 0)continue;
      Vector3f pos5 = new Vector3f(50*j-475,-30f,50*i-475);
      BObject obstracle = new BObject(this,1,box2,pos5,true);
      physics.addBody(obstracle);
      //obstracle.setPosition(new Vector3f(-0,-30,0));
      
      Transform frameInA = new Transform();
      Transform frameInB = new Transform();
      frameInA.setIdentity();
      frameInB.setIdentity();
      
      frameInA.origin.x = pos5.x;
      frameInA.origin.z = pos5.z;
      frameInA.origin.y = -30f;
      
      
      Generic6DofConstraint Fixed = new Generic6DofConstraint(b1.rigidBody,obstracle.rigidBody,frameInA,frameInB,true);
      Vector3f limit = new Vector3f(0f,0f,0f);
      Fixed.setAngularLowerLimit(limit);
      Fixed.setAngularUpperLimit(limit);
      Fixed.setLinearLowerLimit(new Vector3f(0f,0f,0f));
      Fixed.setLinearUpperLimit(new Vector3f(0f,0f,0f));
      physics.addJoint(Fixed);
    }
  }
}  

float angle_x = .0f;
float angle_y = .0f;

public void draw() {
  
  background(0);
  lights();
  
  float angle1 = hinge .getHingeAngle() - angle_x;
  float angle2 = hinge2.getHingeAngle() - angle_y;
  
  hinge.enableAngularMotor(true,  -angle1*2.f, 100000f);
  hinge2.enableAngularMotor(true, -angle2*2.f, 100000f);
  
  physics.update();
  physics.display();
}

public void keyPressed() {
  if (key == CODED) {
    if (keyCode == UP) {
      angle_y -= .05f;
    }
    else if (keyCode == DOWN) {
      angle_y += .05f;
    }
    else if (keyCode == RIGHT) {
      angle_x -= .05f;
    }
    else if (keyCode == LEFT) {
      angle_x += .05f;
    }
  }
  
  if(angle_x > .5f) angle_x = .45f;
  if(angle_x < -.5f) angle_x = -.45f;
  if(angle_y > .5f) angle_y = .45f;
  if(angle_y < -.5f) angle_y = -.45f;
}
