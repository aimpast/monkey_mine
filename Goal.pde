import bRigid.*;
import com.bulletphysics.collision.dispatch.GhostObject;

class Goal {
  public float x;
  public float y;
  public BObject     obj;
  public GhostObject ghost;
  
  Goal(float x,float y,BObject obj,GhostObject ghost) {
    this.x = x;
    this.y = y;
    this.obj = obj;
    this.ghost = ghost;
  }
}
