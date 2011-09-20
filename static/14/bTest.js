var   b2Vec2 = Box2D.Common.Math.b2Vec2
 , b2AABB = Box2D.Collision.b2AABB
 , b2BodyDef = Box2D.Dynamics.b2BodyDef
 , b2Body = Box2D.Dynamics.b2Body
 , b2FixtureDef = Box2D.Dynamics.b2FixtureDef
 , b2Fixture = Box2D.Dynamics.b2Fixture
 , b2World = Box2D.Dynamics.b2World
 , b2MassData = Box2D.Collision.Shapes.b2MassData
 , b2PolygonShape = Box2D.Collision.Shapes.b2PolygonShape
 , b2CircleShape = Box2D.Collision.Shapes.b2CircleShape
 , b2DebugDraw = Box2D.Dynamics.b2DebugDraw
 , b2RevoluteJointDef = Box2D.Dynamics.Joints.b2RevoluteJointDef
 , b2MouseJointDef =  Box2D.Dynamics.Joints.b2MouseJointDef
   ;

function bTest(intervalRate, adaptive, width, height, scale) {
  this.intervalRate = parseInt(intervalRate);
  this.adaptive = adaptive;
  this.width = width;
  this.height = height;
  this.scale = scale;
  
  this.bodiesMap = {};

  this.world = new b2World(
        new b2Vec2(0, 10)    //gravity
     ,  true                 //allow sleep
  );

  this.fixDef = new b2FixtureDef;
  this.fixDef.density = 1.0;
  this.fixDef.friction = 0.5;
  this.fixDef.restitution = 0.2;
}

// bTest.prototype.buildGround = function() {
//   //create ground
//   var bodyDef = new b2BodyDef;
//   bodyDef.type = b2Body.b2_staticBody;
// 
//   // positions the center of the object (not upper left!)
//   bodyDef.position.x = this.width / 2 / this.scale;
//   bodyDef.position.y = this.height / 2 / this.scale;
//   bodyDef.angle = (45*Math.PI) / 180; // radians
//   bodyDef.userData = '__BODY__';
// 
//   this.fixDef.shape = new b2PolygonShape;
// 
//   // half width, half height. eg actual height here is 1 unit
//   this.fixDef.shape.SetAsBox((this.width-(this.width*0.1) / this.scale) / 2, (10/this.scale) / 2);
//   this.registerBody(bodyDef).CreateFixture(this.fixDef);
// }

bTest.prototype.update = function() {
  var start = Date.now();
  var stepRate = (this.adaptive) ? (now - this.lastTimestamp) / 1000 : (1 / this.intervalRate);
  this.world.Step(
         stepRate   //frame-rate
      ,  10       //velocity iterations
      ,  10       //position iterations
   );
   this.world.ClearForces();
   return (Date.now() - start);
}

bTest.prototype.getState = function() {
  var state = {};
  for (var b = this.world.GetBodyList(); b; b = b.m_next) {
    if (b.IsActive() && typeof b.GetUserData() !== 'undefined' && b.GetUserData() != null) {
        state[b.GetUserData()] = this.getBodySpec(b);
    }
  }
  return state;
}

bTest.prototype.getBodySpec = function(b) {
    return {x: b.GetPosition().x, y: b.GetPosition().y, a: b.GetAngle(), c: {x: b.GetWorldCenter().x, y: b.GetWorldCenter().y}};
}

bTest.prototype.setBodies = function(bodyEntities) {
    var bodyDef = new b2BodyDef;
    
    for(var id in bodyEntities) {
        var entity = bodyEntities[id];
        
        if (entity.id == 'ground') {
            bodyDef.type = b2Body.b2_staticBody;
        } else {
            bodyDef.type = b2Body.b2_dynamicBody;
        }
        
        bodyDef.position.x = entity.x;
        bodyDef.position.y = entity.y;
        bodyDef.userData = entity.id;
        bodyDef.angle = entity.angle;
        var body = this.registerBody(bodyDef);
        
        if (entity.radius) {
            this.fixDef.shape = new b2CircleShape(entity.radius);
            body.CreateFixture(this.fixDef);
        } else if (entity.polys) {
            for (var j = 0; j < entity.polys.length; j++) {
                var points = entity.polys[j];
                var vecs = [];
                for (var i = 0; i < points.length; i++) {
                    var vec = new b2Vec2();
                    vec.Set(points[i].x, points[i].y);
                    vecs[i] = vec;
                }
                this.fixDef.shape = new b2PolygonShape;
                this.fixDef.shape.SetAsArray(vecs, vecs.length);
                body.CreateFixture(this.fixDef);
            }
        } else {
            this.fixDef.shape = new b2PolygonShape;
            this.fixDef.shape.SetAsBox(entity.halfWidth, entity.halfHeight);
            body.CreateFixture(this.fixDef);
        }
    }
    this.ready = true;
}

bTest.prototype.registerBody = function(bodyDef) {
    var body = this.world.CreateBody(bodyDef);
    this.bodiesMap[body.GetUserData()] = body;
    return body;
}

bTest.prototype.addRevoluteJoint = function(body1Id, body2Id, params) {
    var body1 = this.bodiesMap[body1Id];
    var body2 = this.bodiesMap[body2Id];
    var joint = new b2RevoluteJointDef();
    joint.Initialize(body1, body2, body1.GetWorldCenter());
    if (params && params.motorSpeed) {
      joint.motorSpeed = params.motorSpeed;
      joint.maxMotorTorque = params.maxMotorTorque;
      joint.enableMotor = true;
    }
    this.world.CreateJoint(joint);
}

bTest.prototype.mouseDownAt = function(x, y) {
  if (!this.mouseJoint) {
     var body = this.getBodyAt(x, y);
     if (body) {
        var md = new b2MouseJointDef();
        md.bodyA = this.world.GetGroundBody();
        md.bodyB = body;
        md.target.Set(x, y);
        md.collideConnected = true;
        md.maxForce = 300.0 * body.GetMass();
        this.mouseJoint = this.world.CreateJoint(md);
        body.SetAwake(true);
     }
  } else {
      this.mouseJoint.SetTarget(new b2Vec2(x, y));
  }
}

bTest.prototype.isMouseDown = function() {
  return (this.mouseJoint != null);
}

bTest.prototype.mouseUp = function() {
  this.world.DestroyJoint(this.mouseJoint);
  this.mouseJoint = null;
}

bTest.prototype.getBodyAt = function(x, y) {
   var mousePVec = new b2Vec2(x, y);
   var aabb = new b2AABB();
   aabb.lowerBound.Set(x - 0.001, y - 0.001);
   aabb.upperBound.Set(x + 0.001, y + 0.001);
   
   // Query the world for overlapping shapes.

   var selectedBody = null;
   this.world.QueryAABB(function(fixture) {
     if(fixture.GetBody().GetType() != b2Body.b2_staticBody) {
        if(fixture.GetShape().TestPoint(fixture.GetBody().GetTransform(), mousePVec)) {
           selectedBody = fixture.GetBody();
           return false;
        }
     }
     return true;
   }, aabb);
   return selectedBody;
}