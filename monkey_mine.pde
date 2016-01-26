

import javax.vecmath.Vector3f;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.GhostObject;
import com.bulletphysics.collision.dispatch.GhostPairCallback;

import peasy.*;
import bRigid.*;

PeasyCam cam;

BPhysics physics;
BBox box;
BBox ground;
GhostObject ghostObject;
BObject r;
BObject g;

public void setup() {
  size(640,480,P3D);
  frameRate(60);
  
  cam = new PeasyCam(this, 600);
  
  Vector3f min = new Vector3f(-1200, -2500, -1200);
  Vector3f max = new Vector3f(1200, 2500, 1200);
  
  physics = new BPhysics(min, max);
  
  physics.world.setGravity(new Vector3f(0,500, 0));
  
  box = new BBox(this, 0, 50, 50, 50);
  
  Vector3f pos = new Vector3f(0,-150,0);
  r = new BObject(this, 1, box, pos, true);
  
  physics.addBody(r);
  
  ground = new BBox(this, 0,500,10,500);
  g = new BObject(this,1,ground,pos,true);
  physics.addBody(g);
  
  //BJointHinge joint = new BJointHinge(g,r,new Vector3f(60,0,0),new Vector3f(60,0,0),new Vector3f(0,1,0),new Vector3f(0,1,0));
  //BJointNail joint2= new BJointNail(g,r);
  //BJoint6DofC joint3= new BJoint6DofC(g,r);
  //physics.addJoint(joint3);
  //physics.addJoint(joint);
  //physics.addJoint(joint2);
  /*
  ground = new BBox(this, 1, new Vector3f(0,200,0), new Vector3f(200, 10, 200), true);
  
  ghostObject = new GhostObject();
  ghostObject.setCollisionShape(ground.collisionShape);
  ghostObject.setCollisionFlags(new CollisionFlags().NO_CONTACT_RESPONSE);
  ghostObject.setWorldTransform(ground.transform);
  physics.world.addCollisionObject(ghostObject);*/
  
}

int i=0;
public void draw() {
  
  background(255);
  lights();
  
  
  physics.update();
  
  Vector3f vec = new Vector3f(50*cos(0.01f*i),50*sin(0.01f*i),0);
  Vector3f vec2= new Vector3f(vec.y,-vec.x,0);
  vec2.normalize();
  r.setRotation(new Vector3f(0,0,1),0.01f*i);
  g.setRotation(new Vector3f(0,0,1),0.01f*i);
  r.setPosition(new Vector3f(50*cos(0.01f*i)+vec2.x*25,-150+50*sin(0.01f*i++)+vec2.y*25,0));
  g.setPosition(new Vector3f(0,-150,0));
  
  //ground.display();
  physics.display();
}
